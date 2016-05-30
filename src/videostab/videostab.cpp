#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <mutex>
#include <chrono>
#include <thread>

using namespace std;
using namespace cv;

const unsigned int THREADS = 8;
const unsigned int SMOOTHING_RADIUS = 60;
#define USE_GPU

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
VideoCapture input;
VideoWriter output;
mutex outputLock;
mutex coutLock;

UMat frames[THREADS];
UMat framesGray[THREADS];
Mat framesOut[THREADS];
bool threadBusy[THREADS];
bool threadWriteLock[THREADS]; //Write locks

const unsigned int FRAMES = max(THREADS + 1, SMOOTHING_RADIUS);
RigidTransform framesT[FRAMES];
RigidTransform framesTTemp[THREADS];
RigidTransform frameZero = RigidTransform();

void calculateDt(const unsigned int i, const unsigned int threadId, bool storeFinal=true)
{
	if (!storeFinal)
	{
		coutLock.lock();
		cout << "Precalculating frame " << i << " (thread " << threadId << ")" << endl;
		coutLock.unlock();
	}

	//Loop around when we are at the end of the array
	const unsigned int idCur = i % FRAMES;
	const unsigned int idPrev = i - 1 % FRAMES;

	//Store into temp matrices
	UMat matCur = frames[threadId % THREADS];
	UMat matPrev = frames[threadId - 1 % THREADS];
	UMat greyCur = framesGray[threadId % THREADS];
	UMat greyPrev = framesGray[threadId - 1 % THREADS];

	//Lock necessary variables
	//framesLock[idCur].lock();
	//framesLock[idPrev].lock();

	//Keypoint vectors and status vectors
	vector <Point2f> _keypointPrev, _keypointCur, keypointPrev, keypointCur;
	vector <uchar> status;
	vector <float> err;

	//Find good features
	goodFeaturesToTrack(greyPrev, _keypointPrev, 500, 0.0005, 3);

	//Calculate optical flow
	calcOpticalFlowPyrLK(greyPrev, greyCur, _keypointPrev, _keypointCur, status, err);

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
	RigidTransform dT(dx, dy, da);

	//Store variables now (used when preprocessing frames)
	if (storeFinal)
		framesT[idCur] = dT;		//Write current transform to final array
	else
		framesTTemp[threadId] = dT; //Write current transform to temp array
}

void analyzeFrame(const unsigned int i, const unsigned int threadId)
{
	coutLock.lock();
	cout << "Analyzing frame " << i << " (thread " << threadId << ")" << endl;
	coutLock.unlock();

	//Loop around when we are at the end of the array
	const unsigned int idCur = i % FRAMES;
	const unsigned int idPrev = i - 1 % FRAMES;

	//Store into temp matrices
	UMat* matCur = &frames[threadId % THREADS];
	UMat* matPrev = &frames[threadId - 1 % THREADS];
	UMat* greyCur = &framesGray[threadId % THREADS];
	UMat* greyPrev = &framesGray[threadId - 1 % THREADS];

	//Calculate transform difference for the frame
	calculateDt(i, threadId);
	RigidTransform dT = framesT[i];

	//Smooth transformation using an averaging window
	RigidTransform T(frameZero);
	//Skip the current frame (we didn't put it into our vector yet)
	for (size_t i = idCur + 1; i < FRAMES + idCur; i++)
	{
		T += framesT[i % FRAMES];
	}
	//Now add in the current frame
	T += dT;
	//Divide by current frame count to get smoothed value
	T /= FRAMES;

	//Create target transformation matrix
	Mat frameTransform = T.makeMatrix();

	//Get output image using affine transformation
	Mat out;
	warpAffine(*matCur, out, frameTransform, matCur->size());

	//Wait for the frame occupying the thread has been written
	while (threadWriteLock[threadId])
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

	//Write values for other frames to use
	framesT[idCur] = dT;		//Write current transform
	framesOut[threadId] = out;	//Write output frame
	threadWriteLock[threadId] = true;

	//Done with frame
	threadBusy[threadId] = false;
}

void startThreads()
{
	//Get and set frame info
	input.set(CV_CAP_PROP_POS_FRAMES, 0); //reset frame position
	const unsigned int maxFrame = input.get(CV_CAP_PROP_FRAME_COUNT);

	//Preprocess first first few frames for transform matrix
	thread threads[THREADS];
	unsigned int threadId = 0;
	for (size_t frame = 1; frame < min(FRAMES, maxFrame) - 1; frame++)
	{
		//Get the frame
		input >> frames[frame];
		cvtColor(frames[frame], framesGray[frame], COLOR_BGR2GRAY);

		//Spawn transform calculation thread
		threads[frame] = thread(calculateDt, frame, (frame - 1) % THREADS, false);
	}
	//Wait for threads to finish
	for (size_t i = 0; i < THREADS; i++)
	{
		threads[i].join();
		threadBusy[i] = true; //set this flag for later
		threadWriteLock[i] = false;
	}

	//Start main threading
	input.set(CV_CAP_PROP_POS_FRAMES, 0);
	unsigned int frame = 1; //next frame to process

	//Start first few threads
	for (frame = 1; frame < min(THREADS, maxFrame - 1); frame++)
	{
		//Spawn transform calculation thread
		threads[frame-1] = thread(analyzeFrame, frame, (frame - 1) % THREADS);
	}
	frame = 1 + THREADS;
	
	//Run all threads, checking for completion
	unsigned int frameWrite = 1; //next frame to write
	unsigned int threadFrameIndex[THREADS];
	while (frame < maxFrame)
	{
		for (int t = 0; t < THREADS; t++)
		{
			if (!threadBusy[t])
			{
				//While we have the next frame to write, do it
				for (int tWrite = 0; tWrite < THREADS; tWrite++)
				{
					if (threadFrameIndex[tWrite] == frameWrite)
					{
						coutLock.lock();
						cout << "Writing frame " << frameWrite << " (thread " << tWrite << ")" << endl;
						coutLock.unlock();
						output.write(framesOut[tWrite]);
						threadWriteLock[tWrite] = false;
						frameWrite++;
					}
				}

				//Launch new frame, if another exists
				if (frame < maxFrame)
				{
					//Read new frame
					input >> frames[frame];
					cvtColor(frames[frame], framesGray[frame], COLOR_BGR2GRAY);

					threadFrameIndex[t] = frame;
					threadBusy[t] = true;
					threads[t] = thread(analyzeFrame, frame, t);
					frame++;
				}
			}
		}
		//Sleep a bit before checking again
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	
	//Wait for all threads to end
	for (size_t i = 0; i < THREADS; i++)
	{
		threads[i].join();
	}
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

	//Prepare input video
	input = VideoCapture(argv[1]);
	assert(input.isOpened());

	//Prepare output video
	output = VideoWriter(argv[2],
		input.get(CV_CAP_PROP_FOURCC),
		input.get(CV_CAP_PROP_FPS),
		Size(input.get(CV_CAP_PROP_FRAME_WIDTH),
			input.get(CV_CAP_PROP_FRAME_HEIGHT)));
	assert(output.isOpened());

	//Prepare analysis variables
	input >> frames[0];
	framesT[0] = RigidTransform();

	//Start threads
	startThreads();

	//Exit
	return 0;
}