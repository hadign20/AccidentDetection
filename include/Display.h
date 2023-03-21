#pragma once
#ifndef _DISPLAY_
#define _DISPLAY_

#include "Track.h"
#include "yolo_utils.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

#define SMOOTH_SIZE				5				//-- trajectory smoothing
#define DRAW_BBOX				true
#define DRAW_TRAJECTORY			false
#define DRAW_LABEL				true

class Display {

public:
	void showTracks(Mat& frame, vector<Track*>& tracks, const std::vector<std::string>& classNames);
	void DrawFilledRect(cv::Mat& frame, const cv::Rect& rect, cv::Scalar cl, int alpha);
	void DrawFilledCircle(cv::Mat& frame, const cv::Point center, const int radius, cv::Scalar cl, double alpha);

private:
	int baseline = 0;
	Scalar Colors[9] = { Scalar(255,0,0),Scalar(0,0,255),Scalar(0,255,0),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127) };
};

#endif // !_DISPLAY_

#pragma once
