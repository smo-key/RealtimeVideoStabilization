#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>
#include <mutex>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>
#include <signal.h>

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
	char ** itr = std::find(begin, end, option);
	if (itr != end && ++itr != end)
	{
		return *itr;
	}
	return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
	return std::find(begin, end, option) != end;
}

#if defined(_MSC_VER) || defined(WIN32)  || defined(_WIN32) || defined(__WIN32__) \
    || defined(WIN64)    || defined(_WIN64) || defined(__WIN64__) 
#define WINDOWS
#elif defined(unix)        || defined(__unix)      || defined(__unix__) \
    || defined(linux)       || defined(__linux)     || defined(__linux__) \
    || defined(sun)         || defined(__sun) \
    || defined(BSD)         || defined(__OpenBSD__) || defined(__NetBSD__) \
    || defined(__FreeBSD__) || defined __DragonFly__ \
    || defined(sgi)         || defined(__sgi) \
    || defined(__MACOSX__)  || defined(__APPLE__) \
    || defined(__CYGWIN__) 
#define UNIX
#endif

using namespace std;

template <typename T>
std::string to_string_with_precision(const T value, const int n = 4)
{
	std::ostringstream out;
	out << std::setprecision(n) << value;
	return out.str();
}

#include <sys/timeb.h>
#if defined(WINDOWS)
#include <windows.h>
bool _qpcInited = false;
double PCFreq = 0.0;
__int64 CounterStart = 0;
void InitCounter()
{
	LARGE_INTEGER li;
	if (!QueryPerformanceFrequency(&li))
	{
		std::cout << "QueryPerformanceFrequency failed!\n";
	}
	PCFreq = double(li.QuadPart) / 1000.0f;
	_qpcInited = true;
}
double CLOCK()
{
	if (!_qpcInited) InitCounter();
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart) / PCFreq;
}

cv::Size GetDesktopResolution()
{
	return cv::Size(GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN));
}
#elif defined(UNIX)
double CLOCK()
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (t.tv_sec * 1000) + (t.tv_nsec*1e-6);
}
#endif

double _avgdur = 0;
double _fpsstart = 0;
double _avgfps = 0;
double _fps1sec = 0;

double avgdur(double newdur)
{
	_avgdur = 0.98*_avgdur + 0.02*newdur;
	return _avgdur;
}

double avgfps()
{
	if (CLOCK() - _fpsstart>1000)
	{
		_fpsstart = CLOCK();
		_avgfps = 0.7*_avgfps + 0.3*_fps1sec;
		_fps1sec = 0;
	}
	_fps1sec++;
	return _avgfps;
}


#define VERBOSE

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
	double norm() {
		return sqrt(x*x + y*y + angle*angle);
	}

	cv::Mat makeTransformationMatrix()
	{
		//Particularly, an affine transformation matrix
		cv::Mat T(2, 3, CV_64F);

		//double _angle = min(max(angle,-CV_PI/45.0), CV_PI/45.0);

		T.at<double>(0, 0) = cos(angle);
		T.at<double>(0, 1) = -sin(angle);
		T.at<double>(1, 0) = sin(angle);
		T.at<double>(1, 1) = cos(angle);

		T.at<double>(0, 2) = x; //x
		T.at<double>(1, 2) = y; //y

		return T;
	}

	cv::Mat makeVector()
	{
		cv::Mat T = cv::Mat(3, 1, CV_32FC1);
		T.at<float>(0, 0) = x;
		T.at<float>(1, 0) = y;
		T.at<float>(2, 0) = angle;
		return T;
	}
};

struct AnalysisResults 
{
public:
	vector <cv::Point2f> keypointPrev;
	vector <cv::Point2f> keypointCur;
	double analysisTime;
	unsigned int thread;

	AnalysisResults()
	{
		keypointPrev = vector<cv::Point2f>();
		keypointCur = vector<cv::Point2f>();
		analysisTime = 0.0;
		this->thread = 0;
	}
	AnalysisResults(unsigned int thread) 
	{
		keypointPrev = vector<cv::Point2f>();
		keypointCur = vector<cv::Point2f>();
		analysisTime = 0.0;
		this->thread = thread;
	}
	AnalysisResults(vector<cv::Point2f> keypointPrev, vector<cv::Point2f> keypointCur, double analysisTime, unsigned int thread)
	{
		this->keypointPrev = keypointPrev;
		this->keypointCur = keypointCur;
		this->analysisTime = analysisTime;
		this->thread = thread;
	}
};

struct StabilizationResults
{
public:
	RigidTransform stateEstimatePriori, stateEstimatePosteriori, measurementResidual, frameCompensation, frameDifference, frameUncorrectedState;
	float errorCovariancePriori, errorCovariancePosteriori, residualCovariance, optimalKalmanGain, stabilityIndex;
	double stabilizationTime;

	StabilizationResults(RigidTransform stateEstPre, RigidTransform stateEstPost, RigidTransform measurementResidual, 
		RigidTransform frameCompensation, RigidTransform frameDifference, RigidTransform frameUncorrectedState,
		float errorCovPre, float errorCovPost, float residCovar, float gain, float stability, double stabilizationTime)
	{
		this->stateEstimatePriori = stateEstPre;
		this->stateEstimatePosteriori = stateEstPost;
		this->measurementResidual = measurementResidual;
		this->frameCompensation = frameCompensation;
		this->frameDifference = frameDifference;
		this->frameUncorrectedState = frameUncorrectedState;
		this->errorCovariancePriori = errorCovPre;
		this->errorCovariancePosteriori = errorCovPost;
		this->residualCovariance = residCovar;
		this->optimalKalmanGain = gain;
		this->stabilityIndex = stability;
		this->stabilizationTime = stabilizationTime;
	}
};

const unsigned int MAXTHREADS = 4;
const unsigned int FRAMEBUFFER = 8;
unsigned int curThreads = MAXTHREADS;

//Kalman filter constants
const double Q_val = 4e-3; //4e-3
const double R_val = 0.25; //0.25
RigidTransform Q(Q_val, Q_val, Q_val); // process noise covariance
RigidTransform R(R_val, R_val, R_val); // measurement noise covariance

//Video I/O
cv::VideoCapture inputAnalyze;
cv::VideoCapture inputStabilize;
cv::VideoWriter output;
mutex coutLock;
ofstream csv;
double videoFps;
bool flipVideoHorizontal;

//Threading
unsigned int threadAnalyzeFrame[MAXTHREADS];	//index of frames each ANALYZE thread is processing
bool threadAnalyzeBusy[MAXTHREADS];				//threads busy processing and cannot be respawned
unsigned int frameAnalyzing = 0;				//current frame being analyzed
unsigned int framesAnalyzed = -1;				//last frame to be analyzed
unsigned int frameStabilizing = 0;				//current frame being stabilized
unsigned int frameWriting = 0;					//current frame being written
bool running = true;							//when false, begin termination
bool terminated = false;						//when true, all threads are done

RigidTransform framesT[FRAMEBUFFER];			//parsed frame transformation deltas
AnalysisResults dataT[FRAMEBUFFER];				//parsed analysis results
cv::Mat frames[FRAMEBUFFER];					//an actual frame buffer between stab and write threads
RigidTransform X, P, K;							//Kalman filter global variables

static void wait(int micros)
{
	this_thread::sleep_for(std::chrono::microseconds(micros));
}

void display(cv::Mat& before, cv::Mat& after, AnalysisResults& analysis, StabilizationResults& stabilization,
	unsigned int frame, unsigned int frameCount)
{
	//Draw some neat debug info
	for (size_t i = 0; i < analysis.keypointPrev.size(); i++)
	{
		line(before, analysis.keypointPrev[i], analysis.keypointCur[i], CV_RGB(0, 255, 0), 2);
		ellipse(before, cv::RotatedRect(analysis.keypointPrev[i], cv::Size2f(5, 5), 0), CV_RGB(255, 255, 0));
	}

	//Draw the original and stablized iamges side-by-side
	cv::Mat canvas = cv::Mat::zeros(before.rows, before.cols * 2 + 10, before.type());

	if (flipVideoHorizontal)
	{
		cv::flip(before, before, 0);
		cv::flip(after, after, 0);
	}

	before.copyTo(canvas(cv::Range::all(), cv::Range(0, before.cols)));
	after.copyTo(canvas(cv::Range::all(), cv::Range(before.cols + 10, before.cols * 2 + 10)));

	//Help info
	cv::putText(canvas, cv::String("Before"), cv::Point(8, 48),
		cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(255, 255, 255), 3);
	cv::putText(canvas, cv::String("After"), cv::Point(8 + (canvas.cols / 2), 48),
		cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(255, 255, 255), 3);

	//Scale canvas if too big
	#ifdef WINDOWS
	while (canvas.cols > GetDesktopResolution().width) {
		resize(canvas, canvas, cv::Size(GetDesktopResolution().width - 16, 
			(int)((double)(GetDesktopResolution().width - 16) * ((double)canvas.rows / (double)canvas.cols))));
	}
	#else
	if (canvas.cols > 1920) {
		resize(canvas, canvas, cv::Size(canvas.cols / 2, canvas.cols / 2));
	}
	#endif

	//Recalculate fps
	cv::putText(canvas, cv::String(to_string_with_precision(avgfps())) + cv::String(" FPS using ")
		+ cv::String(to_string_with_precision(curThreads, 0)) + cv::String(" threads"), cv::Point(8, canvas.rows - 8),
		cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255), 2);
	//Write frame number
	int baseline;
	cv::Size textSize = cv::getTextSize(cv::String("Frame ") + cv::String(to_string_with_precision((int)frame, 0))
		+ cv::String(" of ") + cv::String(to_string_with_precision((int)frameCount, 0)),
		cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
	cv::putText(canvas, cv::String("Frame ") + cv::String(to_string_with_precision((int)frame, 0))
		+ cv::String(" of ") + cv::String(to_string_with_precision((int)frameCount, 0)),
		cv::Point((canvas.cols / 2) - 8 - textSize.width, canvas.rows - 8),
		cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255), 2);
	//Display stability
	cv::putText(canvas, cv::String("Error Covariance: ") + cv::String(to_string_with_precision(stabilization.errorCovariancePosteriori)),
		cv::Point(8 + (canvas.cols / 2), canvas.rows - 8),
		cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255), 2);
	//Display video size
	textSize = cv::getTextSize(cv::String("Video Size: ") + cv::String(to_string_with_precision(before.cols, 0))
		+ cv::String("x") + cv::String(to_string_with_precision(before.rows, 0))
		+ cv::String(" @ ") + cv::String(to_string_with_precision(videoFps, 0)) + cv::String(" fps"),
		cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
	cv::putText(canvas, cv::String("Video Size: ") + cv::String(to_string_with_precision(before.cols, 0))
		+ cv::String("x") + cv::String(to_string_with_precision(before.rows, 0))
		+ cv::String(" @ ") + cv::String(to_string_with_precision(videoFps, 0)) + cv::String(" fps"),
		cv::Point((canvas.cols) - 8 - textSize.width, canvas.rows - 8),
		cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255), 2);

	cv::imshow("Video Stabilization", canvas);
	cv::waitKey(1);

	//Write some info to the csv
	if (csv.is_open())
	{
		csv << frame << "," << to_string_with_precision((double)frame / videoFps) << "," << to_string_with_precision(analysis.analysisTime) << "," <<
			analysis.thread << "," << stabilization.stabilizationTime << "," << analysis.keypointCur.size() << "," <<
			stabilization.frameDifference.x << "," << stabilization.frameDifference.y << "," << stabilization.frameDifference.angle << "," <<
			stabilization.frameUncorrectedState.x << "," << stabilization.frameUncorrectedState.y << "," << stabilization.frameUncorrectedState.angle << "," <<
			stabilization.stateEstimatePriori.x << "," << stabilization.stateEstimatePriori.y << "," << stabilization.stateEstimatePriori.angle << "," <<
			stabilization.stateEstimatePosteriori.x << "," << stabilization.stateEstimatePosteriori.y << "," << stabilization.stateEstimatePosteriori.angle << "," <<
			stabilization.frameCompensation.x << "," << stabilization.frameCompensation.y << "," << stabilization.frameCompensation.angle << "," <<
			stabilization.measurementResidual.x << "," << stabilization.measurementResidual.y << "," << stabilization.measurementResidual.angle << "," <<
			stabilization.errorCovariancePriori << "," << stabilization.errorCovariancePosteriori << "," <<
			stabilization.residualCovariance << "," << stabilization.optimalKalmanGain << "," << stabilization.stabilityIndex << endl;
	}
}

static cv::Mat fisheyeCorrection(cv::Mat& src)
{
	double fx = 857.48296979;
	double fy = 876.71824265;
	double cx = 968.06224829;
	double cy = 556.37145899;

	double distortion[] = { -2.57614020e-1, 8.77086999e-2,
		-2.56970803e-4, -5.93390389e-4, -1.52194091e-2 };

	//Intrinsic matrix
	cv::Mat intrinsics = cv::Mat(3, 3, CV_64FC1);
	intrinsics.setTo(0);
	intrinsics.at<double>(0, 0) = fx;
	intrinsics.at<double>(1, 1) = fy;
	intrinsics.at<double>(2, 2) = 1.0;
	intrinsics.at<double>(0, 2) = cx;
	intrinsics.at<double>(1, 2) = cy;

	//Distortion coefficients
	cv::Mat dist_coeffs = cv::Mat(1, 5, CV_64FC1);
	dist_coeffs.setTo(0);
	dist_coeffs.at<double>(0, 0) = distortion[0];
	dist_coeffs.at<double>(0, 1) = distortion[1];
	dist_coeffs.at<double>(0, 2) = distortion[2];
	dist_coeffs.at<double>(0, 3) = distortion[3];
	dist_coeffs.at<double>(0, 4) = distortion[4];

	cv::Mat dst(src.rows, src.cols, src.type());
	cv::Mat mapx, mapy;

	//Find optimal camera matrix (least cropping)
	cv::Mat cam = getOptimalNewCameraMatrix(intrinsics, dist_coeffs,
		 cv::Size(src.cols, src.rows), 1, cv::Size(), (cv::Rect*)0, true);
	//Prepare undistortion x and y maps
	initUndistortRectifyMap(intrinsics, dist_coeffs, cv::Mat(), cam, cv::Size(src.cols, src.rows), CV_32FC1, mapx, mapy);
	//Perform remap (undistortion) operation
	remap(src, dst, mapx, mapy, cv::INTER_CUBIC);
	undistort(src, dst, intrinsics, dist_coeffs);

	return src;
}

void analyzeFrame(const unsigned int frame, const unsigned int thread, cv::Mat& mat, cv::Mat& prevMat) noexcept
{
	const int id = frame % FRAMEBUFFER;
	double _start = CLOCK();

	cv::Mat grayCur, grayPrev;
	cv::cvtColor(mat, grayCur, cv::COLOR_BGR2GRAY);
	cv::cvtColor(prevMat, grayPrev, cv::COLOR_BGR2GRAY);

	//Keypoint vectors and status vectors
	vector <cv::Point2f> _keypointPrev = vector<cv::Point2f>();
	_keypointPrev.reserve(500);
	vector <cv::Point2f>_keypointCur = vector<cv::Point2f>();
	vector <cv::Point2f> keypointPrev = vector<cv::Point2f>();
	vector <cv::Point2f> keypointCur = vector<cv::Point2f>();

	cv::Mat dst;
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
	clahe->setClipLimit(16); //greater clip limit = more contrast
	clahe->apply(grayCur,grayCur);
	clahe->apply(grayPrev,grayPrev);

	//Find good features
	goodFeaturesToTrack(grayPrev, _keypointPrev, 500, 0.01, 5);

	//Output
	RigidTransform dT(0, 0, 0);
	AnalysisResults results = AnalysisResults(thread);

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

		try
		{
			//Estimate transformation with translation and rotation only
			cv::Mat matT = estimateRigidTransform(keypointPrev, keypointCur, false);
			results.keypointPrev = keypointPrev;
			results.keypointCur = keypointCur;

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
		catch (const std::exception&)
		{
				
		}
	}

	//Finalize output
	framesT[id] = dT;
	results.analysisTime = (CLOCK() - _start) / 1000.0;
	dataT[id] = results;

	//Prepare for another thread
	framesAnalyzed++;
	threadAnalyzeBusy[thread] = false;
}

//Kalman filter
cv::KalmanFilter filter;
cv::Mat measurement;
const unsigned int KALMAN_DIM = 3;

void initKalman() 
{
	//Note: all data uses floats for speed improvement

	//Create Kalman filter
	filter = cv::KalmanFilter(KALMAN_DIM, 3, 0, CV_32F); //(x, y, angle)
	//process noise covariance
	cv::setIdentity(filter.processNoiseCov, cv::Scalar(Q_val));
	//measurement noise covariance
	cv::setIdentity(filter.measurementNoiseCov, cv::Scalar(R_val));
	//a posteriori error covariance matrix
	cv::setIdentity(filter.errorCovPost, cv::Scalar(1));
	//initialize measurement matrix
	cv::setIdentity(filter.measurementMatrix, cv::Scalar(1.0));
	//initialize a posteriori state (initial state)
	cv::setIdentity(filter.statePost, cv::Scalar(0));

	//state = cv::Mat(KALMAN_DIM, 1, CV_32FC1);
	//process_noise = cv::Mat(KALMAN_DIM, 1, CV_32FC1);
	measurement = cv::Mat(KALMAN_DIM, 1, CV_32FC1);
}

/*
Stabilize a frame.

This method is guaranteed to run in series (i.e. all frames in order, and one at a time).
As such, this method should run as quickly as possible. Offload any processing possible
to analyzeFrame();
*/
StabilizationResults stabilizeFrame(const unsigned int frame, RigidTransform frameDiff, RigidTransform frameState, cv::Mat& mat)
{
	double _start = CLOCK();

	//Input:
	//frameDiff = instantaneous difference
	//frameState = overall sum (measurement)
	
	//Predict (a priori)
	const cv::Mat prediction = filter.predict();					//(x, y, angle)
	float errorCovPre = filter.errorCovPre.at<float>(0, 0);			//a priori error covariance
	RigidTransform stateEstPre = RigidTransform(prediction.at<float>(0, 0),
		prediction.at<float>(1, 0), prediction.at<float>(2, 0));	//predicted state estimate

	//Set measurement
	//improve rolling shutter by taking previous frame instead of current frame
	measurement = (frameState - frameDiff).makeVector(); 

	//Correct (a posteriori)
	const cv::Mat target = filter.correct(measurement);
	float gain = filter.gain.at<float>(0, 0);							//optimal Kalman gain
	float errorCovPost = filter.errorCovPost.at<float>(0, 0);			//a posteriori error covariance
	RigidTransform stateEstPost = RigidTransform(target.at<float>(0, 0),
		target.at<float>(1, 0), target.at<float>(2, 0));				//a posteriori state estimate
	RigidTransform measurementResidual = RigidTransform(filter.temp5.at<float>(0, 0),
		filter.temp5.at<float>(1, 0), filter.temp5.at<float>(2, 0));	//measurement residual
	float residCovar = filter.temp3.at<float>(0, 0);					//residual covariance

	/*#define TESTMAT filter.errorCovPost
	float a1 = TESTMAT.at<float>(0, 0);
	float a2 = TESTMAT.at<float>(0, 1);
	float a3 = TESTMAT.at<float>(0, 2);
	float b1 = TESTMAT.at<float>(1, 0);
	float b2 = TESTMAT.at<float>(1, 1);
	float b3 = TESTMAT.at<float>(1, 2);
	float c1 = TESTMAT.at<float>(2, 0);
	float c2 = TESTMAT.at<float>(2, 1);
	float c3 = TESTMAT.at<float>(2, 2);
	float m[9] = { a1, a2, a3, b1, b2, b3, c1, c2, c3 };	
	cout << m << endl;*/

	//Output:
	//stateEstPost		= posteriori state estimate (i.e. result of calculation)
	//frameState		= current measurement

	//Compensate: target - current = compensation (stateEstPost - frameState, frameDiff + stateEstPost - frameState)
	RigidTransform frameCompensation = stateEstPost - frameState;

	//Estimate stability
	RigidTransform _stability = frameCompensation - frameDiff;
	double stability = _stability.norm() != 0 ? 100.0 / (log10(_stability.norm() + 10.0)) : 100.0;

	//Calculate transform matrix
	cv::Mat T = frameCompensation.makeTransformationMatrix();

	//Transform (stabilize) frame
	cv::Mat out;
	cv::warpAffine(mat, out, T, mat.size());
	//Perform fisheye correction
	//out = fisheyeCorrection(out);
	
	//Wait until our framebuffer has an empty slot
	while (frame + FRAMEBUFFER - 1 <= frameStabilizing)
		wait(1);

	//Copy frame to write buffer
	frames[frame % FRAMEBUFFER] = out.clone();

	return StabilizationResults(stateEstPre, stateEstPost, measurementResidual, frameCompensation, frameDiff, frameState,
		errorCovPre, errorCovPost, residCovar, gain, stability, (CLOCK() - _start)/1000.0);
}

void stabilizeFrames(const unsigned int frameCount)
{
	cv::Mat tmp;
	RigidTransform sum(0, 0, 0);

	while (frameStabilizing < frameCount && running)
	{
		//Check lowest frame currently in processing
		int minFrameAnalyzing = INT_MAX;
		bool frameDone = false;
		bool allDone = true;
		for (unsigned int t = 0; t < MAXTHREADS; t++)
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
			StabilizationResults stabResults = stabilizeFrame(frameStabilizing, framesT[frameStabilizing % FRAMEBUFFER],
				sum, tmp);

			//Draw the image on screen
			display(tmp, frames[frameStabilizing % FRAMEBUFFER], dataT[frameStabilizing % FRAMEBUFFER], stabResults, frameStabilizing, frameCount);

			//Prepare for another to spawn
			frameStabilizing++;
		}
		else
		{
			wait(1);
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
		wait(1);
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
	cv::Mat tmp;
	framesT[0] = RigidTransform(0, 0, 0);
	X = RigidTransform(0, 0, 0);
	K = RigidTransform(0, 0, 0);
	P = RigidTransform(1, 1, 1);
	initKalman();

	//Prepare threading
	thread analysisThreads[MAXTHREADS];
	for (size_t i = 0; i < MAXTHREADS; i++)
	{
		threadAnalyzeBusy[i] = false;	//clear this flag so threads can spin up
	}

	//Write in first frame without transformation (for now)
	inputAnalyze >> tmp;
	output.write(tmp);
	cv::Mat prevFrameAnalyzed = tmp.clone();
	cv::Mat t1, t2;

	//Start stabiliziation thread
	thread stabThread = thread(stabilizeFrames, frameCount);

	//Start frame writer
	thread writeThread = thread(writeFrames, frameCount);

	unsigned int curMaxThread = 0;

	//Run all threads until everything is written
	while ((frameStabilizing < frameCount) && running)
	{
		for (int t = 0; t < MAXTHREADS; t++)
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

				//Check how many threads we are using once in a while
				if ((frameAnalyzing % MAXTHREADS * 5) <= MAXTHREADS * 3)
				{
					curMaxThread = max(curMaxThread, t);
				}
				if ((frameAnalyzing % MAXTHREADS * 5) == MAXTHREADS * 3)
				{
					curThreads = curMaxThread + 1;
					curMaxThread = 0;
				}

				//Spawn new analysis thread
				t1 = tmp.clone();
				t2 = prevFrameAnalyzed.clone();

				analysisThreads[t] = thread(analyzeFrame, frameAnalyzing, t, std::ref(t1), std::ref(t2));

				//Prepare for another to spawn
				tmp.copyTo(prevFrameAnalyzed);
				frameAnalyzing++;

				break;
			}
		}
		wait(1);
	}

	cout << "Waiting for threads..." << endl;

	//Wait for threads to terminate
	if (stabThread.joinable())
		stabThread.join();
	if (writeThread.joinable())
		writeThread.join();
	for (size_t i = 0; i < MAXTHREADS; i++)
	{
		if (analysisThreads[i].joinable())
			analysisThreads[i].join();
	}

	cout << "Terminated!" << endl;

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
	#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	while (!terminated)
	 	wait(5000);
	#endif

	cout << "All threads closed!" << endl;

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
		cout << "videostab [-f] [-c] [-d dataout.csv] input_video output_video" << endl << endl;
		cout << "Choose Codec (-c): Show manual codec selection if available" << endl;
		cout << "Output Data  (-d): Output analysis and stabilization data as CSV" << endl;
		cout << "Flip Stream  (-f): Flip video horizontally for some buggy codecs" << endl;
	}

	flipVideoHorizontal = cmdOptionExists(argv, argv + argc, "-f");

	//Catch escape signals
	//signal(SIGABRT, &handleSignal);
	signal(SIGTERM, &handleSignal);
	signal(SIGINT, &handleSignal);

	//Prepare input video streams
	inputAnalyze = cv::VideoCapture(argv[argc-2]);
	assert(inputAnalyze.isOpened());

	inputStabilize = cv::VideoCapture(argv[argc-2]);
	assert(inputStabilize.isOpened());

	//Prepare video stream
	videoFps = inputAnalyze.get(CV_CAP_PROP_FPS);
	cv::Size videoSize = cv::Size(inputAnalyze.get(CV_CAP_PROP_FRAME_WIDTH),
		inputAnalyze.get(CV_CAP_PROP_FRAME_HEIGHT));
    output = cv::VideoWriter(argv[argc-1],
	cmdOptionExists(argv, argv+argc, "-c") ? -1 : CV_FOURCC('D', 'I', 'V', 'X'), videoFps, videoSize);
	assert(output.isOpened());

	if (argc >= 4 && sizeof(argv[argc - 3]) / sizeof(char) > 3)
	{
		csv = ofstream(argv[argc - 3]);
		assert(csv.is_open());
		csv << "Video Width," << videoSize.width << endl
			<< "Video Height," << videoSize.height << endl
			<< "Video FPS" << videoFps << endl
			<< "Input Filename" << argv[argc - 2] << endl << endl;
		csv << "Frame,Video Time,Analysis Time,Analysis Thread,Stabilization Time,Keypoints Found," <<
			"Delta X,Delta Y,Delta Theta,Uncorrected X,Uncorrected Y,Uncorrected Theta," <<
			"State Estimate Priori X,State Estimate Priori Y,State Estimate Priori Theta," <<
			"Corrected X (State Estimate Posteriori),Corrected Y (State Estimate Posteriori),Corrected Theta (State Estimate Posteriori)," <<
			"Compensation X,Compensation Y,Compensation Theta," <<
			"Measurement Residual X,Measurement Residual Y,Measurement Residual Theta," <<
			"Error Covariance Priori P(k|k-1),Error Covariance Posteriori P(k|k)," <<
			"Residual Covariance (S),Optimal Kalman Gain (K),Stability Index (%)" << endl;
	}

	//Main operation
	startThreads();

	//Exit!
	cout << "Done! Closing files and exiting..." << endl;
	inputAnalyze.release();
	inputStabilize.release();
	output.release();
	csv.close();
	return 0;
}
