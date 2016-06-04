#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <mutex>
#include <chrono>
#include <thread>
#include <signal.h>

using namespace std;
using namespace cv;

const unsigned int THREADS = 6;
const unsigned int FRAMEBUFFER = 16;

struct RigidTransform
{
public:
	double x, y;	//position
	double angle;	//rotation

	RigidTransform()
	{
		x = 0.0;
		y = 0.0;
		angle = 0.0;
	}
	RigidTransform(double x, double y, double angle)
	{
		this->x = x;
		this->y = y;
		this->angle = angle;
	}
	RigidTransform& operator+=(const RigidTransform &other) {
		this->x += other.x;
		this->y += other.y;
		this->angle += other.angle;
		return *this;
	}
	RigidTransform& operator-=(const RigidTransform &other) {
		this->x -= other.x;
		this->y -= other.y;
		this->angle -= other.angle;
		return *this;
	}
	RigidTransform& operator/=(const int &other) {
		this->x /= other;
		this->y /= other;
		this->angle /= other;
		return *this;
	}
	friend RigidTransform operator+(const RigidTransform &c1, const RigidTransform  &c2) {
		return RigidTransform(c1.x + c2.x, c1.y + c2.y, c1.angle + c2.angle);
	}
	friend RigidTransform operator-(const RigidTransform &c1, const RigidTransform  &c2) {
		return RigidTransform(c1.x - c2.x, c1.y - c2.y, c1.angle - c2.angle);
	}
	friend RigidTransform operator*(const RigidTransform &c1, const RigidTransform  &c2) {
		return RigidTransform(c1.x*c2.x, c1.y*c2.y, c1.angle*c2.angle);
	}
	friend RigidTransform operator/(const RigidTransform &c1, const RigidTransform  &c2) {
		return RigidTransform(c1.x / c2.x, c1.y / c2.y, c1.angle / c2.angle);
	}

	Mat makeMatrix()
	{
		//Particularly, an affine transformation matrix
		Mat T(2, 3, CV_64F);

		T.at<double>(0, 0) = cos(angle);
		T.at<double>(0, 1) = -sin(angle);
		T.at<double>(1, 0) = sin(angle);
		T.at<double>(1, 1) = cos(angle);

		T.at<double>(0, 2) = x;
		T.at<double>(1, 2) = y;

		return T;
	}
};

//Kalman filter constants
const double Q_val = 4e-3; //4e-3
const double R_val = 0.25; //0.25
RigidTransform Q(Q_val, Q_val, Q_val); // process noise covariance
RigidTransform R(R_val, R_val, R_val); // measurement noise covariance 

//Video I/O
VideoCapture inputAnalyze;
VideoCapture inputStabilize;
VideoWriter output;
mutex coutLock;

//Threading
unsigned int threadAnalyzeFrame[THREADS];	//index of frames each ANALYZE thread is processing
bool threadAnalyzeBusy[THREADS];			//threads busy processing and cannot be respawned
unsigned int frameAnalyzing = 0;			//current frame being analyzed
unsigned int framesAnalyzed = -1;			//last frame to be analyzed
unsigned int frameStabilizing = 0;			//current frame being stabilized
unsigned int frameWriting = 0;				//current frame being written
bool running = true;						//when false, begin termination
bool terminated = false;					//when true, all threads are done

RigidTransform framesT[FRAMEBUFFER];			//parsed frame transformation deltas
Mat frames[FRAMEBUFFER];						//an actual frame buffer between stab and write threads
RigidTransform X, P, K;							//Kalman filter global variables

static void wait(int millis)
{
	this_thread::sleep_for(std::chrono::milliseconds(millis));
}

void analyzeFrame(const unsigned int frame, const unsigned int thread, Mat& mat, Mat& prevMat)
{
	const int id = frame % FRAMEBUFFER;

	Mat grayCur, grayPrev;
	cvtColor(mat, grayCur, COLOR_BGR2GRAY);
	cvtColor(prevMat, grayPrev, COLOR_BGR2GRAY);

	//Keypoint vectors and status vectors
	vector <Point2f> _keypointPrev = vector<Point2f>();
	_keypointPrev.reserve(500);
	vector <Point2f>_keypointCur = vector<Point2f>();
	vector <Point2f> keypointPrev = vector<Point2f>();
	vector <Point2f> keypointCur = vector<Point2f>();

	//Find good features
	goodFeaturesToTrack(grayPrev, _keypointPrev, 500, 0.0005, 3);

	//Output
	RigidTransform dT(0, 0, 0);

	if (_keypointPrev.size() > 0)
	{
		vector <uchar> status = vector<uchar>();
		vector <float> err = vector<float>();

		//Calculate optical flow
		calcOpticalFlowPyrLK(grayPrev, grayCur, _keypointPrev, _keypointCur, status, err);

		//Remove bad matches
		for (size_t i = 0; i < status.size(); i++)
		{
			if (status[i])
			{
				keypointPrev.push_back(_keypointPrev[i]);
				keypointCur.push_back(_keypointCur[i]);
			}
		}

		//Estimate transformation with translation and rotation only
		Mat matT = estimateRigidTransform(keypointPrev, keypointCur, false);
		
		//Sometimes, no transformation could be found
		if ((matT.rows == 2) && (matT.cols == 3))
		{
			//Decompose transformation matrix
			double dx = matT.at<double>(0, 2);
			double dy = matT.at<double>(1, 2);
			double da = atan2(matT.at<double>(1, 0), matT.at<double>(0, 0));
			dT = RigidTransform(dx, dy, da);
		}
	}

	//Finalize output
	framesT[id] = dT;

	//Prepare for another thread
	framesAnalyzed++;
	threadAnalyzeBusy[thread] = false;
}

/*
Stabilize a frame.

This method is guaranteed to run in series (i.e. all frames in order, and one at a time).
As such, this method should run as quickly as possible. Offload any processing possible
to analyzeFrame();
*/
void stabilizeFrame(const unsigned int frame, RigidTransform t_i, RigidTransform Z, Mat& mat)
{
	//Kalman filter local variables
	RigidTransform P_, X_;

	//X		= posteriori state estimate (i.e. result of calculation)
	//X_	= priori state estimate
	//P		= posteriori estimate error covariance
	//P_	= priori estiate error covariance
	//K		= gain
	//Z		= current measurement

	//Update Kalman filter predictions
	X_ = X;									//X_(k) = X(k-1);
	P_ = P + Q;								//P_(k) = P(k-1)+Q;

	//Measurement correction
	K = P_ / (P_ + R);						//K(k) = P_(k)/( P_(k)+R );
	X = X_ + K*(Z - X_);					//z-X_ is residual, X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
	P = (RigidTransform(1, 1, 1) - K)*P_;	//P(k) = (1-K(k))*P_(k);

	//Compensate: now + (target - current)
	RigidTransform dT = t_i + X - Z;

	//Calculate transform matrix
	Mat T = dT.makeMatrix();

	//Transform frame
	Mat out;
	warpAffine(mat, out, T, mat.size());

	//Wait until our framebuffer has an empty slot
	while (frame + FRAMEBUFFER - 1 <= frameStabilizing)
		wait(1);

	//Copy frame to write buffer
	frames[frame % FRAMEBUFFER] = out.clone();
}

void stabilizeFrames(const unsigned int frameCount)
{
	Mat tmp;
	RigidTransform sum(0, 0, 0);

	while (frameStabilizing < frameCount && running)
	{
		//Launch when necessary frame was analyzed
		if (framesAnalyzed >= frameStabilizing)
		{
			//Get another frame from video
			inputStabilize >> tmp;

			//Write some debug info :)
			coutLock.lock();
			cout << "Stabilizing frame " << frameStabilizing << endl;
			coutLock.unlock();

			//Get measured input
			sum += framesT[frameStabilizing % FRAMEBUFFER];

			//Stabilize frame - run *quickly*
			stabilizeFrame(frameStabilizing, framesT[frameStabilizing % FRAMEBUFFER],
				sum, tmp);

			//Prepare for another to spawn
			frameStabilizing++;	
		}
		wait(5);
	}
}

void writeFrames(const unsigned int frameCount)
{
	while (frameWriting < frameCount && running)
	{
		//Wait until we have a frame to write
		if (frameStabilizing - 1 >= frameWriting)
		{
			coutLock.lock();
			cout << "Writing     frame " << frameWriting << endl;
			coutLock.unlock();
			output << frames[frameWriting % FRAMEBUFFER];
			frameWriting++;
		}
		wait(5);
	}
}

void startThreads()
{
	//Get frame count
	const unsigned int frameCount = inputAnalyze.get(CV_CAP_PROP_FRAME_COUNT);
	cout << "Starting analysis of " << frameCount << " frames..." << endl;

	//Reset input buffer positions
	inputAnalyze.set(CV_CAP_PROP_POS_FRAMES, 0);	//reset buffer position
	inputStabilize.set(CV_CAP_PROP_POS_FRAMES, 1);  //reset buffer position
	frameAnalyzing = 1;								//we skip the first frame
	frameStabilizing = 1;							//we write the first frame without stabilizing
	framesAnalyzed = 0;
	frameWriting = 1;								//we write the first frame early

	//Set initial variables
	Mat tmp;
	framesT[0] = RigidTransform(0, 0, 0);
	X = RigidTransform(0, 0, 0);
	K = RigidTransform(0, 0, 0);
	P = RigidTransform(1, 1, 1);

	//Prepare threading
	thread analysisThreads[THREADS];
	for (size_t i = 0; i < THREADS; i++)
	{
		threadAnalyzeBusy[i] = false;	//clear this flag so threads can spin up
	}

	//Write in first frame without transformation (for now)
	inputAnalyze >> tmp;
	output.write(tmp);
	Mat prevFrameAnalyzed = tmp.clone();

	//Start stabiliziation thread
	thread stabThread = thread(stabilizeFrames, frameCount);

	//Start frame writer
	thread writeThread = thread(writeFrames, frameCount);

	//Run all threads until everything is written
	while (frameStabilizing < frameCount && running)
	{
		for (int t = 0; t < THREADS; t++)
		{
			//Check on analysis threads
			//Make sure that the thread is not busy AND more frames are left to analyze AND
			//running it won't overlap any data
			if ((!threadAnalyzeBusy[t]) && (frameAnalyzing < frameCount) &&
				(frameWriting + FRAMEBUFFER - 1 > frameAnalyzing))
			{
				//Make sure thread is actually done
				if (analysisThreads[t].joinable())
					analysisThreads[t].join();

				//Set frame vars
				threadAnalyzeFrame[t] = frameAnalyzing;
				threadAnalyzeBusy[t] = true;

				//Get another frame from video
				inputAnalyze >> tmp;

				//Write some debug info :)
				coutLock.lock();
				cout << "Analyzing   frame " << frameAnalyzing << " (thread " << t << ")" << endl;
				coutLock.unlock();

				//Spawn new analysis thread
				analysisThreads[t] = thread(analyzeFrame, frameAnalyzing, t, tmp.clone(), prevFrameAnalyzed.clone());

				//Prepare for another to spawn
				tmp.copyTo(prevFrameAnalyzed);
				frameAnalyzing++;
			}
		}
		wait(1);
	}

	//Wait for threads to terminate
	if (stabThread.joinable())
		stabThread.join();
	if (writeThread.joinable())
		writeThread.join();
	for (size_t i = 0; i < THREADS; i++)
	{
		if (analysisThreads[i].joinable())
			analysisThreads[i].join();
	}

	terminated = true;
}

void handleSignal(int sig)
{
	cout << "Signal " << sig << " caught. Exiting safely..." << endl;

	//Raise exit flag
	running = false;

	//Wait for threads to terminate
	while (!terminated)
		wait(50);

	//Close streams
	output.release();
	inputAnalyze.release();
	inputStabilize.release();

	//Exit
	exit(0);
}

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		cout << "./videostab (options) (data.csv) [input.mp4] [output.mp4]" << endl;
		cout << endl;
		cout << "TRANSFORM OPTIONS" << endl;
		cout << "-S (-prsh)  Similarity Transform (Position, Rotation, Scale, sHear)" << endl;
		cout << endl;
		cout << "RENDERING OPTIONS" << endl;
		cout << "-G          Use GPU (default=no)" << endl;
		cout << "-Tn         Use n threads (default=4)" << endl;
	}

	//Catch escape signals
	//signal(SIGABRT, &handleSignal);
	signal(SIGTERM, &handleSignal);
	signal(SIGINT, &handleSignal);

	//Prepare input video streams
	inputAnalyze = VideoCapture(argv[1]);
	assert(inputAnalyze.isOpened());

	inputStabilize = VideoCapture(argv[1]);
	assert(inputStabilize.isOpened());

	//Prepare output video stream
	output = VideoWriter(argv[2],
		inputAnalyze.get(CV_CAP_PROP_FOURCC),
		inputAnalyze.get(CV_CAP_PROP_FPS),
		Size(inputAnalyze.get(CV_CAP_PROP_FRAME_WIDTH),
			inputAnalyze.get(CV_CAP_PROP_FRAME_HEIGHT)));
	assert(output.isOpened());

	//Main operation
	startThreads();

	//Exit!
	cout << "Done! Closing files and exiting..." << endl;
	inputAnalyze.release();
	inputStabilize.release();
	output.release();
	return 0;
}