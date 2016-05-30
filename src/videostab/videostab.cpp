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

const unsigned int THREADS = 8;
const unsigned int SMOOTHING_RADIUS = 5;
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

Mat framesOut[THREADS];
bool threadBusy[THREADS];
bool threadWriteLock[THREADS]; //Write locks

const unsigned int FRAMES = max(THREADS, SMOOTHING_RADIUS*2);
Mat frames[FRAMES];
Mat framesGray[FRAMES];
RigidTransform framesT[FRAMES];
RigidTransform frameZero = RigidTransform();

void calculateDt(const unsigned int i, const unsigned int threadId, bool storeFinal=true)
{
	//Loop around when we are at the end of the array
	const int idCur = i % FRAMES;
	int idPrev = (idCur - 1);
	idPrev = (idPrev < 0) ? FRAMES - 1 : idPrev;

	if (!storeFinal)
	{
		coutLock.lock();
		cout << "Precalculating frame " << i << " (thread " << threadId << ") ID: " << idPrev << "," << idCur << endl;
		coutLock.unlock();
	}

	//Store into temp matrices
	Mat matCur = frames[idCur];
	Mat matPrev = frames[idPrev];
	Mat greyCur = framesGray[idCur];
	Mat greyPrev = framesGray[idPrev];

	//Lock necessary variables
	//framesLock[idCur].lock();
	//framesLock[idPrev].lock();

	//Keypoint vectors and status vectors
	vector <Point2f> _keypointPrev, _keypointCur
		, keypointPrev, keypointCur;

	//Find good features
	goodFeaturesToTrack(greyPrev, _keypointPrev, 500, 0.0005, 3);

	//Output
	RigidTransform dT;

	if (_keypointPrev.size() != 0)
	{
		vector <uchar> status = vector<uchar>();
		vector <float> err = vector<float>();

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
		dT = RigidTransform(dx, dy, da);
	}
	else
	{
		dT = RigidTransform();
	}

	//Store variables now (used when preprocessing frames)
	if (storeFinal)
	{
		framesT[idCur] = dT;		//Write current transform to final array
	}
	else
	{
		framesT[idCur] = dT; //Write current transform to temp array
		threadBusy[threadId] = false;
	}
}

void analyzeFrame(const unsigned int i, const unsigned int threadId)
{
	//Loop around when we are at the end of the array
	int idCur = i % FRAMES;
	int idPrev = (idCur - 1);
	idPrev = (idPrev < 0) ? FRAMES - 1 : idPrev;

	coutLock.lock();
	//cout << "Analyzing frame " << i << " (thread " << threadId << ") ID: " << idPrev << "," << idCur << endl;
	coutLock.unlock();

	//Store into temp matrices
	Mat* matCur = &frames[idCur];
	Mat* matPrev = &frames[idPrev];
	Mat* greyCur = &framesGray[idCur];
	Mat* greyPrev = &framesGray[idPrev];

	//Calculate transform difference for the frame
	if (i >= FRAMES)
		calculateDt(i, threadId);
	RigidTransform dT = framesT[idCur];
	if (idCur == 0)
		frameZero = dT;

	//Smooth transformation using an averaging window
	RigidTransform T(frameZero);
	//Skip the current frame (we didn't put it into our vector yet)
	for (size_t i = idCur + 1; i < FRAMES + idCur; i++)
	{
		T += framesT[i % FRAMES + 1];
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

	coutLock.lock();
	//cout << "Wait      frame " << i << " (thread " << threadId << ") ID: " << idPrev << "," << idCur << endl;
	coutLock.unlock();

	//Wait for the frame occupying the thread has been written
	while (threadWriteLock[threadId])
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

	//Write values for other frames to use
	framesT[idCur] = dT;		//Write current transform
	framesOut[threadId] = out;	//Write output frame
	threadWriteLock[threadId] = true;

	//Done with frame
	threadBusy[threadId] = false;

	coutLock.lock();
	//cout << "Done      frame " << i << " (thread " << threadId << ") ID: " << idPrev << "," << idCur << endl;
	coutLock.unlock();
}

void startThreads()
{
	//Get and set frame info
	input.set(CV_CAP_PROP_POS_FRAMES, 0); //reset frame position
	const unsigned int maxFrame = input.get(CV_CAP_PROP_FRAME_COUNT);

	//Prepare threading variables
	unsigned int frame = 1; //next frame to process
	thread threads[THREADS];
	for (size_t i = 0; i < THREADS; i++)
	{
		threadBusy[i] = false;		//clear this flag so threads can spin up
		threadWriteLock[i] = false; //open write lock flag so first frames can be written
	}

	//Read in first frame
	input >> frames[0];
	cvtColor(frames[0], framesGray[0], COLOR_BGR2GRAY);

	//Preprocess first first few frames for transform matrix
	while (frame < FRAMES)
	{
		for (int t = 0; t < THREADS; t++)
		{
			if (!threadBusy[t])
			{
				//Launch new frame, if another exists
				if (frame < FRAMES)
				{
					if (threads[t].joinable())
						threads[t].join();

					//Read new frame
					input >> frames[frame];
					cvtColor(frames[frame], framesGray[frame], COLOR_BGR2GRAY);

					threadBusy[t] = true;
					threads[t] = thread(calculateDt, frame, t, false);
					frame++;
				}
			}
		}
		//Sleep a bit before checking again
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	//Wait for threads to finish, then reset vars again
	unsigned int threadFrameIndex[THREADS];
	for (size_t i = 0; i < THREADS; i++)
	{
		threads[i].join();
		threadBusy[i] = false;		//unset flag so we can analyze data
		threadWriteLock[i] = false; //open write lock flag so first frames can be written
		threadFrameIndex[i] = -1;
	}

	//Start main threading
	input.set(CV_CAP_PROP_POS_FRAMES, 0);

	//Read in first radius of frames
	for (int f = 0; f < FRAMES / 2; f++)
	{
		input >> frames[f];
		cvtColor(frames[f], framesGray[f], COLOR_BGR2GRAY);
	}

	frame = 1;
	
	//Run all threads, checking for completion
	unsigned int frameWrite = 1; //next frame to write
	while (frameWrite < maxFrame)
	{
		for (int t = 0; t < THREADS; t++)
		{
			//While we have the next frame to write, do it
			if (threadFrameIndex[t] == frameWrite && threadWriteLock[t])
			{
				coutLock.lock();
				cout << "Writing   frame " << frameWrite << " (thread " << t << ")" << endl;
				coutLock.unlock();
				output.write(framesOut[t]);
				threadWriteLock[t] = false;
				frameWrite++;
			}

			if (!threadBusy[t] && !threadWriteLock[t])
			{
				//Process new frame, if another exists
				int framePos = (frame + (FRAMES / 2)) % FRAMES;
				if (frame < maxFrame)
				{
					//Make sure thread is done
					if (threads[t].joinable())
						threads[t].join();

					//Read new frame if available
					if (framePos < maxFrame)
					{
						input >> frames[framePos];
						cvtColor(frames[framePos], framesGray[framePos], COLOR_BGR2GRAY);
					}
					
					//Launch thread
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
		if (threads[i].joinable())
			threads[i].join();
		//threads[i].detach();
	}
	return;
}

void handleSignal(int sig)
{
	cout << "Signal " << sig << " caught. Exiting safely..." << endl;

	//Close streams
	/*input.release();
	output.release();

	terminate();
	exit(0);*/
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

	//Close streams
	input.release();
	output.release();

	//Exit
	return 0;
}