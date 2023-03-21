#pragma once
#ifndef _TRACKER_
#define _TRACKER_

#include "Kalman.h"
#include "HungarianAlg.h"
#include "Track.h"
#include "Intersection.h"


class Multitracker {
public:
	Multitracker();
	~Multitracker();
	vector<Track*> tracks;
	void update_tracks(std::vector<Detection>& detections, Intersection inters, int front);
	void lane_change(string timestamp, ofstream& file);
	void countV(string timestamp, Mat& frame, ofstream& file, Intersection& GIS_map);
	void conflict_detection(string timestamp, vector<std::string> classNames, Mat& frame, ofstream& file, Mat& GIS);

private:
	float acceleration = 1;
	int max_distance = 2500;
	int max_misses = 5;
	int max_trace = 60;
	float dt = 0.2;

};


#endif //_TRACKER_
