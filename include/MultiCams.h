#pragma once
#ifndef _MULTICAMS_
#define _MULTICAMS_


#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

class MultiCams{

public:

	MultiCams(string configPath);
	~MultiCams();
	vector<string> video_addresses;
	vector<Mat> tMatrix;
	Ptr<BackgroundSubtractorMOG2> MOG_FG = createBackgroundSubtractorMOG2(500, 16, false);
	vector<int> front_point;
	vector<vector<Point>> rois;
	vector<string> cam_info;
	vector<int> hours;
	vector<int> minutes;
	vector<int> seconds;

};




#endif 

#pragma once
