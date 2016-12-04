/*
 Copyright (C) 2016 Arthur Pachachura
 MIT licensed. Share and reuse!
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	if (argc < 3) {
		cout << "./SideBySide [before.mp4] [after.mp4]" << endl;
		return 0;
	}

	// Setup inputAnalyze video
	VideoCapture before(argv[1]);
	assert(before.isOpened());

	VideoCapture after(argv[2]);
	assert(after.isOpened());

	Mat cur_before, cur_after;

	int k = 0;
	int max_frames = before.get(CV_CAP_PROP_FRAME_COUNT);

	//before.set(CV_CAP_PROP_POS_FRAMES, 0);
	//after.set(CV_CAP_PROP_POS_FRAMES, 0);

	while (k < max_frames - 1) { // don't process the very last frame, no valid transform
		before >> cur_before;
		after >> cur_after;

		if (cur_before.data == NULL) {
			break;
		}
		if (cur_after.data == NULL) {
			break;
		}

		// Now draw the original and stablised side by side for coolness
		Mat canvas = Mat::zeros(cur_before.rows, cur_before.cols * 2 + 10, cur_before.type());

		cur_before.copyTo(canvas(Range::all(), Range(0, cur_after.cols)));
		cur_after.copyTo(canvas(Range::all(), Range(cur_after.cols + 10, cur_after.cols * 2 + 10)));

		// If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
		while (canvas.cols > 1920) {
			resize(canvas, canvas, Size(canvas.cols / 1.5, canvas.rows / 1.5));
		}

		imshow("Before and After", canvas);

		waitKey(1000 / 120);

		k++;
	}

	return 0;
}