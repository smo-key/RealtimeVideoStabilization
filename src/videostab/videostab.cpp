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
	RigidTransform& operator/=(const int &other) {
		this->x /= other;
		this->y /= other;
		this->angle /= other;
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

const unsigned int FRAMES = SMOOTHING_RADIUS * 2;	//2*frames needed for a single frame to be ready to write
const unsigned int FRAMEBUFFER = FRAMES + THREADS;	//frame buffer size
RigidTransform framesT[FRAMEBUFFER];				//parsed frame transformation deltas
RigidTransform framesSumT[FRAMEBUFFER];				//sum of all frames in position from previous framesT
													//framesT is reset to zeroed every overlap, but values are added to framesSumT

/*
New algorithm:

In parallel:
a. Find transform of threadMat (THREADS at a time) up to current frame needed -> framesT
b. Stabilize threadMat (THREADS at a time)
	1. Wait until all threadAnalyzeFrames are above FRAMES_ANALYZED + frame being processed
	2. Stabilize frame
	3. Wait for this frame to be the next to write (check frameStabilizing)
	4. Write frame to output
*/

static void wait(int millis)
{
	this_thread::sleep_for(std::chrono::milliseconds(millis));
}

void analyzeFrame(const unsigned int frame, const unsigned int thread, Mat mat, Mat prevMat)
{
	const int id = frame % FRAMEBUFFER;

	Mat grayCur, grayPrev;
	cvtColor(mat, grayCur, COLOR_BGR2GRAY);
	cvtColor(prevMat, grayPrev, COLOR_BGR2GRAY);

	//Keypoint vectors and status vectors
	vector <Point2f> _keypointPrev, _keypointCur
		, keypointPrev, keypointCur;

	//Find good features
	goodFeaturesToTrack(grayPrev, _keypointPrev, 500, 0.0005, 3);

	//Output
	RigidTransform dT;

	if (_keypointPrev.size() != 0)
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

		//Decompose transformation matrix
		double dx = matT.at<double>(0, 2);
		double dy = matT.at<double>(1, 2);
		double da = atan2(matT.at<double>(1, 0), matT.at<double>(0, 0));
		dT = RigidTransform(dx, dy, da);
	}
	else
	{
		dT = RigidTransform();
	}

	//Finalize output
	framesT[id] = dT;

	//Prepare for another thread
	framesAnalyzed++;
	threadAnalyzeBusy[thread] = false;
}

void stabilizeFrame(const unsigned int frame, const unsigned int thread, Mat mat)
{
	//Find the accumulated dT
	RigidTransform transform = framesT[frame % FRAMEBUFFER];

	//Calculate transform matrix
	Mat T = transform.makeMatrix();

	//Transform frame
	warpAffine(mat, mat, T, mat.size());

	//Wait for thread write lock to release
	while (threadWriteLock[thread])
		wait(5);

	//Write out transformation
	framesStabilizeOut[thread] = mat;

	//Prepare for another thread
	threadWriteLock[thread] = true;			//Wait to write
	threadStabilizeBusy[thread] = false;
}

void writeFrames(const unsigned int frameCount)
{
	while (frameWriting < frameCount)
	{
		for (int t = 0; t < THREADS; t++)
		{
			//While we have the next frame to write, do it
			if ((threadStabilizeFrame[t] == frameWriting) && (threadWriteLock[t]))
			{
				coutLock.lock();
				cout << "Writing     frame " << frameWriting << endl;
				coutLock.unlock();
				output << framesStabilizeOut[t];
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
	for (int f = 0; f < FRAMEBUFFER; f++)
	{
		framesSumT[f] = RigidTransform(0, 0, 0);
	}

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
	output << tmp;
	Mat prevFrameAnalyzed = tmp;

	//Start frame writer
	thread writeThread = thread(writeFrames, frameCount);

	//Run all threads until everything is written
	while (frameStabilizing < frameCount)
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
				analysisThreads[t] = thread(analyzeFrame, frameAnalyzing, t, tmp, prevFrameAnalyzed);

				//Prepare for another to spawn
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
				stabilizeThreads[t] = thread(stabilizeFrame, frameStabilizing, t, tmp);

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
}

void handleSignal(int sig)
{
	cout << "Signal " << sig << " caught. Exiting safely..." << endl;

	//Close streams
	inputAnalyze.~VideoCapture();
	output.~VideoWriter();

	terminate();
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

	//Main operation
	startThreads();

	//Exit!
	cout << "Done! Closing files and exiting..." << endl;
	inputAnalyze.~VideoCapture();
	inputStabilize.~VideoCapture();
	output.~VideoWriter();
	return 0;
}