//#define USE_CUDA

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <mutex>
#include <chrono>
#include <thread>
#include <signal.h>

#ifdef USE_CUDA
#include "opencv2/cudalegacy.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudawarping.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/cudabgsegm.hpp"
#include "opencv2/cudacodec.hpp"
#endif

using namespace std;
using namespace cv;

const unsigned int THREADS = 2;
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

#ifdef USE_CUDA
template<class T>
static vector<T> copyGMatToVector(cuda::GpuMat& _mat)
{
	Mat mat;

	//Copy data from GPU to CPU
	_mat.download(mat);

	//Pointer to the i-th row
	const T* p = mat.ptr<T>(0);

	//Copy data to a vector.  Note that (p + mat.cols) points to the
	//end of the row.
	vector<T> vec(p, p + mat.cols);

	return vec;
}
#endif

static void wait(int millis)
{
	this_thread::sleep_for(std::chrono::milliseconds(millis));
}

static void display(Mat& before, Mat& after)
{
	//Sraw the original and stablized iamges side-by-side
	Mat canvas = Mat::zeros(before.rows, before.cols * 2 + 10, before.type());

	before.copyTo(canvas(Range::all(), Range(0, before.cols)));
	after.copyTo(canvas(Range::all(), Range(before.cols + 10, before.cols * 2 + 10)));

	//Scale canvas if too big
	if (canvas.cols > 1920) {
		resize(canvas, canvas, Size(canvas.cols / 2, canvas.rows / 2));
	}

	imshow("Before and After", canvas);
	waitKey(1);
}

#ifndef USE_CUDA
static Mat fisheyeCorrection(Mat& src)
#else
static cuda::GpuMat fisheyeCorrection(cuda::GpuMat& src)
#endif
{
	double fx = 857.48296979;
	double fy = 876.71824265;
	double cx = 968.06224829;
	double cy = 556.37145899;

	double distortion[] = { -2.57614020e-1, 8.77086999e-2,
		-2.56970803e-4, -5.93390389e-4, -1.52194091e-2 };
	
	//Intrinsic matrix
	Mat intrinsics = Mat(3, 3, CV_64FC1);
	intrinsics.setTo(0);
	intrinsics.at<double>(0, 0) = fx;
	intrinsics.at<double>(1, 1) = fy;
	intrinsics.at<double>(2, 2) = 1.0;
	intrinsics.at<double>(0, 2) = cx;
	intrinsics.at<double>(1, 2) = cy;

	//Distortion coefficients
	Mat dist_coeffs = Mat(1, 5, CV_64FC1);
	dist_coeffs.setTo(0);
	dist_coeffs.at<double>(0, 0) = distortion[0];
	dist_coeffs.at<double>(0, 1) = distortion[1];
	dist_coeffs.at<double>(0, 2) = distortion[2];
	dist_coeffs.at<double>(0, 3) = distortion[3];
	dist_coeffs.at<double>(0, 4) = distortion[4];

#ifndef USE_CUDA
	Mat dst(src.rows, src.cols, src.type());
#else
	cuda::GpuMat dst(src.rows, src.cols, src.type());
#endif

	Mat mapx, mapy;

	//Find optimal camera matrix (least cropping)
	Mat cam = getOptimalNewCameraMatrix(intrinsics, dist_coeffs, 
		Size(src.cols, src.rows), 1, Size(), (Rect*)0, true);
	//Prepare undistortion x and y maps
	initUndistortRectifyMap(intrinsics, dist_coeffs, Mat(), cam, Size(src.cols, src.rows), CV_32FC1, mapx, mapy);
	//Perform remap (undistortion) operation
#ifndef USE_CUDA
	remap(src, dst, mapx, mapy, INTER_LINEAR);
#else
	cuda::GpuMat _mapx(mapx), _mapy(mapy);
	cuda::remap(src, dst, _mapx, _mapy, INTER_LINEAR);
#endif
	//undistort(src, dst, intrinsics, dist_coeffs);

	return dst;
}

void analyzeFrame(const unsigned int frame, const unsigned int thread, Mat& mat, Mat& prevMat)
{
	const int id = frame % FRAMEBUFFER;

#ifndef USE_CUDA
	Mat grayCur, grayPrev;

	cvtColor(mat, grayCur, COLOR_BGR2GRAY);
	cvtColor(prevMat, grayPrev, COLOR_BGR2GRAY);
#else
	cuda::GpuMat grayCur, grayPrev, _mat(mat), _prevMat(prevMat);

	cuda::cvtColor(_mat, grayCur, COLOR_BGR2GRAY);
	cuda::cvtColor(_prevMat, grayPrev, COLOR_BGR2GRAY);
#endif

	//Keypoint vectors and status vectors
	vector <Point2f> _keypointPrev;
	vector <Point2f> _keypointCur;

	//Find good features
#ifndef USE_CUDA
	_keypointPrev = vector<Point2f>();
	_keypointPrev.reserve(500);
	//_keypointCur = vector<Point2f>();
	goodFeaturesToTrack(grayPrev, _keypointPrev, 500, 0.0005, 3);
#else
	cuda::GpuMat _keypointPrevMat;
	cuda::createGoodFeaturesToTrackDetector(grayPrev.type(), 500, 0.0005, 3)->detect(grayPrev, _keypointPrevMat);

	// Copy data from _keypointPrevMat to _keypointPrev vector
	_keypointPrev = copyGMatToVector<Point2f>(_keypointPrevMat);
#endif

	//Output
	RigidTransform dT(0, 0, 0);

	if (_keypointPrev.size() > 0)
	{
		vector <uchar> status;
		vector <float> err;

		//Calculate optical flow
#ifndef USE_CUDA
		calcOpticalFlowPyrLK(grayPrev, grayCur, _keypointPrev, _keypointCur, status, err);
#else
		cuda::GpuMat _status, _err, _keypointCurMat;
		cuda::SparsePyrLKOpticalFlow::create()->calc(grayPrev, grayCur, _keypointPrevMat, _keypointCurMat, _status, _err);

		status = copyGMatToVector<uchar>(_status);
		err = copyGMatToVector<float>(_err);

		_keypointCur = copyGMatToVector<Point2f>(_keypointCurMat);
		_status.release();
		_err.release();
		_keypointCurMat.release();
		_keypointPrevMat.release();
#endif

		vector <Point2f> keypointPrev = vector<Point2f>();
		vector <Point2f> keypointCur = vector<Point2f>();

		//Remove bad matches
		for (size_t i = 0; i < status.size(); i++)
		{
			if (status[i])
			{
				keypointPrev.push_back(_keypointPrev[i]);
				keypointCur.push_back(_keypointCur[i]);
			}
		}

		Mat kP(keypointPrev), kC(keypointCur);

		//Estimate transformation with translation and rotation only
		Mat matT = estimateRigidTransform(kP, kC, false);
		
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

#ifndef USE_CUDA
	//Perform fisheye correction
	Mat out1 = fisheyeCorrection(mat);

	//Transform (stabilize) frame
	Mat out;
	warpAffine(out1, out, T, mat.size());
#else
	//Perform fisheye correction
	cuda::GpuMat cudaMat(mat);
	cuda::GpuMat out1 = fisheyeCorrection(cudaMat);

	//Transform (stabilize) frame
	cuda::GpuMat out;
	cuda::warpAffine(out1, out, T, cudaMat.size());
#endif

	//Wait until our framebuffer has an empty slot
	while (frame + FRAMEBUFFER - 1 <= frameStabilizing)
		wait(1);

	//Copy frame to write buffer
#ifndef USE_CUDA
	frames[frame % FRAMEBUFFER] = out.clone();
#else
	Mat _out(out);
	frames[frame % FRAMEBUFFER] = _out.clone();
	out.release();
	out1.release();
#endif
}

void stabilizeFrames(const unsigned int frameCount)
{
	Mat tmp;
	RigidTransform sum(0, 0, 0);

	while (frameStabilizing < frameCount && running)
	{
		//Check lowest frame currently in processing
		int minFrameAnalyzing = INT_MAX;
		bool frameDone = false;
		bool allDone = true;
		for (unsigned int t = 0; t < THREADS; t++)
		{
			if (threadAnalyzeBusy[t]) 
				allDone = false;
			if ((threadAnalyzeFrame[t] == frameStabilizing) &&
				(!threadAnalyzeBusy[t]))
			{
				frameDone = true;
				break;
			}
			if (threadAnalyzeFrame[t] < minFrameAnalyzing)
				minFrameAnalyzing = threadAnalyzeFrame[t];
		}
		if (minFrameAnalyzing >= frameStabilizing + 1 || allDone)
			frameDone = true;

		//Launch when necessary frame was analyzed
		if (frameDone)
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

			//Draw the image on screen
			display(tmp, frames[frameStabilizing % FRAMEBUFFER]);

			//Prepare for another to spawn
			frameStabilizing++;	
		}
		else
		{
			wait(5);
		}
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
	coutLock.lock();
	cout << "Signal " << sig << " caught. Exiting safely..." << endl;
	coutLock.unlock();

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
		VideoWriter::fourcc('X','2','6','4'),
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