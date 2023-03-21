#pragma once
#ifndef _INTERSECTION_
#define _INTERSECTION_

#define _THROUGH_LANE 0
#define _LEFT_TURN_LANE 1
#define _RIGHT_TURN_LANE 2


#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>



using namespace cv;
using namespace std;



struct Approach {
    int approach_id;
    int n_lanes;
    int speed_limit;
    vector<int> lane_types;
    vector<vector<Point>> lanes;
    vector<vector<Point>> trajectories;
    vector<Point> stop_bar;
    vector<int> current_v;
    vector<int> total_v;


};

class Intersection
{
public:
    int map_height = 1080;
    int map_width = 1920;
    float ftpp = 0;
    int n_approaches;
    int int_id;
    vector<Approach> approaches;
    Mat lane_map;
    Mat bg;
    vector<Point> references;
    vector<Point2f> GPS_coors;

    Intersection();
    ~Intersection();

    void loadConfig(string configPath);
    void drawIntersection(Mat& frame);
};


#endif 
