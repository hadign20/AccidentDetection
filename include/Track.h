#pragma once
#ifndef _TRACK_
#define _TRACK_

#include "Kalman.h"
#include "HungarianAlg.h"
#include "yolo_utils.h"
#include "Intersection.h"


#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

class Track
{
public:
	vector<Point2f> trace;
	vector<Point2d> bird_trace;
	vector<Point2f> future_trace;
	vector<int> lane;
	vector<float>dirs;
	vector<Point2f>movements;
	vector<float>dist2StopBar;
	static int NextID;
	int track_id;
	int misses;
	int predict_frames = 60;
	Point2f prediction;
	cv::Rect box;
	cv::Rect orig_box;
	std::vector<cv::Point> outline;
	float conf;
	int classId;
	TKalmanFilter* KF;
	float velocity; //mph
	Point2f speed = Point2f(0.0,0.0);
	bool lane_change=false;
	Track(Point2f p, float dt, float acceleration, Intersection inters);
	~Track();
	void UpdateTrack(Point2f center, cv::Rect current_box, Intersection inters);
	float point2line(Point p, vector<Point> l);
	void predictFuture();
};


#endif //_TRACK_

