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

const unsigned int THREADS = 4;
const unsigned int SMOOTHING_RADIUS = 10;

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
	RigidTransform operator+(const RigidTransform &other) {
		this->x += other.x;
		this->y += other.y;
		this->angle += other.angle;
		return *this;
	}
	RigidTransform operator-(const RigidTransform &other) {
		this->x -= other.x;
		this->y -= other.y;
		this->angle -= other.angle;
		return *this;
	}
	RigidTransform operator*(const int &other) {
		this->x *= other;
		this->y *= other;
		this->angle *= other;
		return *this;
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

//Video I/O
VideoCapture inputAnalyze;
VideoCapture inputStabilize;
VideoWriter output;
mutex coutLock;

//Threading
unsigned int threadAnalyzeFrame[THREADS];	//index of frames each ANALYZE thread is processing
unsigned int threadStabilizeFrame[THREADS];	//index of frames each STABILIZE thread is processing
Mat framesStabilizeOut[THREADS];						//output from each thread
bool threadAnalyzeBusy[THREADS];			//threads busy processing and cannot be respawned
bool threadStabilizeBusy[THREADS];			//threads busy processing and cannot be respawned
bool threadWriteLock[THREADS];				//STABILIZE threads waiting to write new frame
unsigned int frameAnalyzing = 0;			//current frame being analyzed
unsigned int framesAnalyzed = -1;			//last frame to be analyzed
unsigned int frameStabilizing = 0;			//current frame being stabilized
unsigned int frameWriting = 0;				//current frame being written
bool running = true;						//when false, begin termination
bool terminated = false;					//when true, all threads are done

const unsigned int FRAMES = SMOOTHING_RADIUS * 2;	//2*frames needed for a single frame to be ready to write
const unsigned int FRAMEBUFFER = FRAMES + THREADS;	//frame buffer size
RigidTransform framesT[FRAMEBUFFER];				//parsed frame transformation deltas (biased)
RigidTransform framesTOrig[FRAMEBUFFER];			//parsed frame transformation deltas (original)
RigidTransform framesTBias;							//sum of all frames in position from previous framesT
													//framesT is reset to zeroed every overlap, but values are added to framesTBias
mutex framesBiasLock;								//when locked, make sure no one can access the sum framesT

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
	vector <Point2f> _keypointPrev = vector<Point2f>(), _keypointCur = vector<Point2f>()
		, keypointPrev = vector<Point2f>(), keypointCur = vector<Point2f>();

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
		Mat matT = estimateRigidTransform(keypointPrev, keypointCur, false); //false = rigid

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
	framesBiasLock.lock();
	framesT[id] = dT;
	framesTOrig[id] = dT;

	if (id == 0) {
		//We need to flush the buffer and update it with equivalent values
		//This allows us to keep track of an overall sum, not just a dT

		//First, make sure no one else is reading from it
		
		RigidTransform sumT(0, 0, 0);
		for (int i = 0; i < FRAMEBUFFER; i++)
		{
			sumT += framesT[i];
		}

		framesTBias += sumT;
		/*for (int i = 0; i < FRAMEBUFFER; i++)
		{
			framesT[i] -= dT;
		}*/
	}
	framesBiasLock.unlock();

	//Prepare for another thread
	framesAnalyzed++;
	threadAnalyzeBusy[thread] = false;
}

void stabilizeFrame(const unsigned int frameCount, const unsigned int frame, const unsigned int thread, Mat& mat)
{
	//Find frames
	const unsigned int frameMin = frame < SMOOTHING_RADIUS ? 0 : frame - SMOOTHING_RADIUS;
	const unsigned int frameMax = frame + SMOOTHING_RADIUS > frameCount ? frameCount : frame + SMOOTHING_RADIUS;  //we're *HOPEFULLY* not going to overflow this
	const unsigned int frameReset = (framesAnalyzed - (framesAnalyzed % FRAMEBUFFER)); //floor to nearest 24

	//Get buffer indices
	const unsigned int idMin = frameMin % FRAMEBUFFER;
	const unsigned int idMax = frameMax % FRAMEBUFFER;

	//Get dT for the current frame
	framesBiasLock.lock(); //make sure no one else is writing
	RigidTransform t_i = framesTOrig[frame % FRAMEBUFFER];

	//Accumulate dT over entire time span to get a linear trajectory for the frame
	RigidTransform t_linear(0, 0, 0);
	t_linear += framesTBias; //start with the bias and add up all frames until now
	if (frame < frameReset)
	{
		//The bias reset already happened, so our frame starts near the end
		//We essentially subtract the leftovers of the bias because those frames are excluded
		for (int i = frame % FRAMEBUFFER; i < FRAMEBUFFER; i++)
		{
			t_linear -= framesT[i];
		}
	}
	else
	{
		for (int i = 0; i <= frame % FRAMEBUFFER; i++)
		{
			t_linear += framesT[i];
		}
	}

	//Average dT over the select time span only to get a smoothed trajectory
	RigidTransform t_smoothed(0, 0, 0);
	if (idMax < idMin)
	{
		//Two (non-overlapping) ranges
		for (int i = idMin; i < FRAMEBUFFER; i++)
		{
			t_smoothed += framesTOrig[i];
		}
		for (int i = 0; i <= idMax; i++)
		{
			t_smoothed += framesTOrig[i];
		}
	}
	else
	{
		//Single (typical) range
		//for (int)
		for (int i = idMin; i <= idMax; i++)
		{
			t_smoothed += framesTOrig[i];
		}
	}
	t_smoothed /= (2 * SMOOTHING_RADIUS);
	framesBiasLock.unlock(); //we're done, so unlock

	//Find the difference between linear and smoothed versions to get a transform function for this frame
	RigidTransform t = t_i + t_smoothed - t_linear;

	//Find the accumulated dT
	//RigidTransform transform = framesT[frame % FRAMEBUFFER];

	//Calculate transform matrix
	Mat T = t.makeMatrix();

	//Transform frame
	Mat out;
	warpAffine(mat, out, T, mat.size());

	//Wait for thread write lock to release
	while (threadWriteLock[thread])
		wait(5);

	//Write out transformation
	framesStabilizeOut[thread] = out;

	//Prepare for another thread
	threadWriteLock[thread] = true;			//Wait to write
	threadStabilizeBusy[thread] = false;
}

void writeFrames(const unsigned int frameCount)
{
	while (frameWriting < frameCount && running)
	{
		for (int t = 0; t < THREADS; t++)
		{
			//While we have the next frame to write, do it
			if ((threadStabilizeFrame[t] == frameWriting) && (threadWriteLock[t]))
			{
				coutLock.lock();
				cout << "Writing     frame " << frameWriting << endl;
				coutLock.unlock();
				output.write(framesStabilizeOut[t]);
				threadWriteLock[t] = false;
				frameWriting++;
			}
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
	framesTOrig[0] = RigidTransform(0, 0, 0);
	framesTBias = RigidTransform(0, 0, 0);

	//Prepare threading
	thread analysisThreads[THREADS];
	thread stabilizeThreads[THREADS];
	for (size_t i = 0; i < THREADS; i++)
	{
		threadAnalyzeBusy[i] = false;	//clear this flag so threads can spin up
		threadStabilizeBusy[i] = false;	//clear this flag so threads can spin up
		threadWriteLock[i] = false;		//open write lock flag so first threadMat can be written
	}
	
	//Write in first frame without transformation (for now)
	inputAnalyze >> tmp;
	output.write(tmp);
	Mat prevFrameAnalyzed = tmp.clone();

	//Start frame writer
	thread writeThread = thread(writeFrames, frameCount);

	//Run all threads until everything is written
	while (frameStabilizing < frameCount && running)
	{
		for (int t = 0; t < THREADS; t++)
		{
			//Check on ANALYSIS threads
			//Make sure that the thread is not busy AND more frames are left to analyze AND
			//running it won't overlap any data
			if ((!threadAnalyzeBusy[t]) && (frameAnalyzing < frameCount) &&
				(frameStabilizing + FRAMEBUFFER >= frameAnalyzing))
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

			//Check on STABILIZE threads
			//Make sure that the thread is not busy AND more frames are left to stabilize AND
			//analysis has completed enough analysis to continue
			if ((!threadStabilizeBusy[t]) && (frameStabilizing < frameCount) && 
				//thread has no write locks on it
				(!threadWriteLock[t]) && 
				//either analysis is ahead enough of stabilization OR analysis reached the end
				((framesAnalyzed >= frameStabilizing + FRAMES) || (framesAnalyzed == frameCount - 1)))
			{
				//Make sure thread is actually done
				if (stabilizeThreads[t].joinable())
					stabilizeThreads[t].join();

				//Set frame vars
				threadStabilizeFrame[t] = frameStabilizing;
				threadStabilizeBusy[t] = true;

				//Get another frame from video
				inputStabilize >> tmp;

				//Write some debug info :)
				coutLock.lock();
				cout << "Stabilizing frame " << frameStabilizing << " (thread " << t << ")" << endl;
				coutLock.unlock();

				//Launch thread
				stabilizeThreads[t] = thread(stabilizeFrame, frameCount, frameStabilizing, t, tmp.clone());

				//Prepare for another to spawn
				frameStabilizing++;
			}
		}
		wait(1);
	}

	//Wait for threads to terminate
	if (writeThread.joinable())
		writeThread.join();
	for (size_t i = 0; i < THREADS; i++)
	{
		if (analysisThreads[i].joinable())
			analysisThreads[i].join();
		if (stabilizeThreads[i].joinable())
			stabilizeThreads[i].join();
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