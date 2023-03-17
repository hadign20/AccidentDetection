#include "include/Track.h"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
int Track::NextID = 0;

Track::Track(Point2f pt, float dt, float acceleration, Intersection inters)
{
	track_id = NextID;
	NextID++;
	KF = new TKalmanFilter(pt, dt, acceleration);
	prediction = pt;
	misses = 0;
	velocity = 0;
	classId = 0;
	conf = 0.0;
	trace.push_back(prediction);


	int current_lane = inters.lane_map.at<Vec3b>(pt.y, pt.x)[0];
	lane.push_back(current_lane);

	float dist;
	if (current_lane == 0 || current_lane == 255)
		dist = 0;
	else {
		vector<Point> stopBar = inters.approaches[current_lane / 10].stop_bar;
		dist = point2line(pt, stopBar) * inters.ftpp;
	}
	dist2StopBar.push_back(dist);

}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
Track::~Track()
{
	delete KF;
}

void Track::UpdateTrack(Point2f center, cv::Rect current_box, Intersection inters) {

	box = current_box;
	Point2f last_center = trace[trace.size() - 1];
	trace.push_back(center);
	Point2f movement;
	//cout << center << ",," << last_center << endl;
	movement.x = center.x - last_center.x;
	movement.y = center.y - last_center.y;
	//cout << movement << endl;
	//float movement = sqrt(1.0*pow(last_center.x - center.x, 2) + pow(last_center.y - center.y, 2));
	movements.push_back(movement);
	//find current lane
	int current_lane = inters.lane_map.at<Vec3b>(center.y, center.x)[0];
	lane.push_back(current_lane);

	//calculater the distance to stopbar
	float dist;
	if (current_lane == 0 || current_lane == 255)
		dist = 0;
	else {
		vector<Point> stopBar = inters.approaches[current_lane / 10].stop_bar;
		dist = point2line(center, stopBar) * inters.ftpp;
	}
	dist2StopBar.push_back(dist);

	predictFuture();
	return;
}

float Track::point2line(Point p, vector<Point> l) {
	float a = l[0].y - l[1].y;
	float b = l[1].x - l[0].x;
	float c = l[0].x * l[1].y - l[1].x * l[0].y;
	return abs(a * p.x + b * p.y + c) / sqrt(a * a + b * b);
}

void Track::predictFuture() {

	if (trace.size() > 10) {
		future_trace.clear();
		Point current_location = trace[trace.size() - 1];
		Point2f location = current_location;
		float move_x = 0.0, move_y = 0.0, d_x = 0.0,d_y = 0.0;
		for (int i = 0; i < 10; i++) {
			//cout << movements[movements.size() - i - 1] << endl;
			move_x += movements[movements.size() - i - 1].x;
			move_y += movements[movements.size() - i - 1].y;
			//d_x = d_x + movements[movements.size() - i - 1].x - movements[movements.size() - i - 2].x;
			//d_y = d_y + movements[movements.size() - i - 1].y - movements[movements.size() - i - 2].y;
		}
		move_x = move_x / 10;
		move_y = move_y / 10;
		speed.x = move_x;
		speed.y = move_y;
		/*d_x = movements[movements.size() - 1].x + movements[movements.size() - 2].x + movements[movements.size() - 3].x + movements[movements.size() - 4].x + movements[movements.size() - 5].x -
			(movements[movements.size() - 6].x + movements[movements.size() - 7].x + movements[movements.size() - 8].x + movements[movements.size() - 9].x + movements[movements.size() - 10].x);
		d_x = d_x / 5;
		d_y = movements[movements.size() - 1].y + movements[movements.size() - 2].y + movements[movements.size() - 3].y + movements[movements.size() - 4].y + movements[movements.size() - 5].y -
			(movements[movements.size() - 6].y + movements[movements.size() - 7].y + movements[movements.size() - 8].y + movements[movements.size() - 9].y + movements[movements.size() - 10].y);
		d_y = d_y / 5;*/
		for (int i = 0; i < predict_frames; i++) {
			//int x = trace[trace.size() - 10 + i % 10].x - trace[trace.size() - 11 + i % 10].x;
			//int y = trace[trace.size() - 10 + i % 10].y - trace[trace.size() - 11 + i % 10].y;
			//cout << move_x << "y: " << move_y << ", d_x" << d_x << ", dy " << d_y << endl;
			location.x += move_x;
			location.y += move_y;
			//move_x += d_x;
			//move_y += d_y;
			future_trace.push_back(location);
		}

	}
	////keep the magnitute and angle speed
	//if (bird_trace.size() > 10) {
	//	future_trace.clear();
	//	Point current_location = bird_trace[bird_trace.size() - 1];
	//	Point location = current_location;
	//	float v_x = (bird_trace[bird_trace.size() - 6].x - bird_trace[bird_trace.size() - 1].x);
	//	float v_y = (bird_trace[bird_trace.size() - 6].y - bird_trace[bird_trace.size() - 1].y);
	//	float v_x_p = (bird_trace[bird_trace.size() - 11].x - bird_trace[bird_trace.size() - 6].x);
	//	float v_y_p = (bird_trace[bird_trace.size() - 11].y - bird_trace[bird_trace.size() - 6].y);
	//	float theta, d_theta, mag;
	//	if (v_x == 0) {
	//		theta = atan(v_y / 0.001);
	//	}
	//	else
	//		theta = atan(v_y / v_x);
	//	if (v_x_p == 0)
	//		d_theta = (atan(v_y_p / 0.001) - theta)/5.0;
	//	else
	//		d_theta = (atan(v_y_p / v_x_p) - theta) / 5.0;
	//	mag = sqrt(pow(v_x, 2) + pow(v_y, 2))/5.0;

	//	for (int i = 0; i < predict_frames; i++) {
	//		theta -= d_theta;
	//		int d_x = mag * cos(theta);
	//		int d_y = mag * sin(theta);
	//		location.x += d_x;
	//		location.y += d_y;
	//		future_trace.push_back(location);
	//	}

	//}


	////kalman filter
	//if (bird_trace.size() > 2) {
	//	future_trace.clear();
	//	Point start_location = bird_trace[0];
	//	Point current_location = bird_trace[bird_trace.size() - 1];
	//	TKalmanFilter* pred_KF = new TKalmanFilter(start_location, 1, 0.1);
	//	for (int i = 1; i < bird_trace.size(); i++) {
	//		Mat prediction = pred_KF->kalman->predict();
	//		Mat measurement(2, 1, CV_32FC1);
	//		measurement.at<float>(0) = bird_trace[i].x;  //update using measurements
	//		measurement.at<float>(1) = bird_trace[i].y;
	//		Mat estimated = pred_KF->kalman->correct(measurement);
	//	}
	//	for (int i = 0; i < predict_frames; i++) {
	//		Mat prediction = pred_KF->kalman->predict();
	//		Mat measurement(2, 1, CV_32FC1);
	//		measurement.at<float>(0) = prediction.at<float>(0);  //update using measurements
	//		measurement.at<float>(1) = prediction.at<float>(1);
	//		Mat estimated = pred_KF->kalman->correct(measurement);
	//		future_trace.push_back(Point(estimated.at<float>(0), estimated.at<float>(1)));
	//	}

	//	delete pred_KF;
	//}
	return;
}
