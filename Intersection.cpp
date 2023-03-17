#include "include/Intersection.h"

Intersection::Intersection() 
{

}
Intersection::~Intersection()
{

}

void Intersection::loadConfig(string configPath) {
    FileStorage fs(configPath, FileStorage::READ);


    if (!fs["height"].isNone())
        fs["height"] >> map_height;
    if (!fs["height"].isNone())
        fs["width"] >> map_width;
    if (!fs["ftpp"].isNone())
        fs["ftpp"] >> ftpp;
    lane_map = Mat(map_height, map_width, CV_8UC3, Scalar(0, 0, 0));
    bg = Mat(map_height, map_width, CV_8UC3, Scalar(0, 0, 0));
    if (!fs["bg"].isNone()) {
        string bg_img;
        fs["bg"] >> bg_img;
        bg = imread(bg_img, cv::IMREAD_COLOR);
        map_height = bg.rows;
        map_width = bg.cols;
        //imshow("bg", bg);
    }
    float latitude = 0, longitude = 0;
    fs["lat1"] >> latitude;
    fs["long1"] >> longitude;
    GPS_coors.push_back(Point2f(latitude, longitude));
    fs["lat2"] >> latitude;
    fs["long2"] >> longitude;
    GPS_coors.push_back(Point2f(latitude, longitude));



    fs["n_approaches"] >> n_approaches; //number of approaches
    for (int i = 0; i < n_approaches; i++) {  //for each approach
        Approach ap;
        ap.approach_id = i;
        fs[("approach" + to_string(i) + "_lanes").c_str()] >> ap.n_lanes;   //number of lanes
        fs[("approach" + to_string(i) + "_sl").c_str()] >> ap.speed_limit;  //speed limit
        //location of the stop bar
        ap.stop_bar.push_back(Point(fs[("approach" + to_string(i) + "_stop_x0").c_str()], fs[("approach" + to_string(i) + "_stop_y0").c_str()]));
        ap.stop_bar.push_back(Point(fs[("approach" + to_string(i) + "_stop_x1").c_str()], fs[("approach" + to_string(i) + "_stop_y1").c_str()]));


        for (int j = 0; j < ap.n_lanes; j++) {    //for each lane
            ap.lane_types.push_back(fs[("approach" + to_string(i) + "_lane" + to_string(j) + "_type").c_str()]);    //lane type
            vector<Point> region; //the contour points of the lane
            vector<Point> t; //trajectory 
            int k = 0;
            while (1) {
                if (fs[("approach" + to_string(i) + "_lane" + to_string(j) + "_x" + to_string(k)).c_str()].isNone())
                    break;
                else {
                    region.push_back(Point(fs[("approach" + to_string(i) + "_lane" + to_string(j) + "_x" + to_string(k)).c_str()], fs[("approach" + to_string(i) + "_lane" + to_string(j) + "_y" + to_string(k)).c_str()]));
                    k++;
                }
            }
            k = 0;
            while (1) {
                if (fs[("approach" + to_string(i) + "_t" + to_string(j) + "_x" + to_string(k)).c_str()].isNone())
                    break;
                else {
                    t.push_back(Point(fs[("approach" + to_string(i) + "_t" + to_string(j) + "_x" + to_string(k)).c_str()], fs[("approach" + to_string(i) + "_t" + to_string(j) + "_y" + to_string(k)).c_str()]));
                    k++;
                }
            }

            ap.lanes.push_back(region);
            ap.trajectories.push_back(t);
            int val = i * 10 + j+1;
            fillPoly(lane_map, region, Scalar(val, val, val));

            ap.current_v.push_back(0);
            ap.total_v.push_back(0);
        }
        approaches.push_back(ap);
    }
    fs.release();
    drawIntersection(lane_map);
    return;


}

void Intersection::drawIntersection(Mat& frame) {
    for (int i = 0; i < n_approaches; i++) {
        Approach ap = approaches[i];
        cv::line(frame, ap.stop_bar[0], ap.stop_bar[1], cv::Scalar(0, 255, 0), 2);
        for (int j = 0; j < ap.n_lanes; j++) {
            vector<Point> region = ap.lanes[j];
            int val = i * 10 + j + 1;
            if (ap.lane_types[j] == _THROUGH_LANE)
                polylines(frame, region, 1, cv::Scalar(val, 50, 50), 2, 8, 0);
            else if (ap.lane_types[j] == _LEFT_TURN_LANE)
                polylines(frame, region, 1, cv::Scalar(val, 128, 128), 2, 8, 0);
            else if (ap.lane_types[j] == _RIGHT_TURN_LANE)
                polylines(frame, region, 1, cv::Scalar(val, 128, 60), 2, 8, 0);
            else
                ;
        }
    }
    return;


}
