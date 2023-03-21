#pragma once
#ifndef _SPEEDESTIMATOR_
#define _SPEEDESTIMATOR_

#include "Track.h"

class SpeedEstimator {
public:
	void detectSpeed(vector<Track*>& tracks, double fps, float ftpp); // calculates speed in mph
private:
	int counter = 0;
	int interval = 15;
	int update_freq = 15; //-- number of frames to pass to update the speed - the higher the value the smoother the updating
	int lane_width = 35; //-- number of pixels corresponding to lane width (12 feet or 0.00227 miles)
};
#endif // !_SPEEDESTIMATOR_
