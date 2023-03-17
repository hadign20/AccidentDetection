#include "include/SpeedEstimator.h"

void SpeedEstimator::detectSpeed(vector<Track*>& tracks, double fps, float ftpp) {
	for (int i = 0; i < tracks.size(); i++)
	{
		//cout << tracks[i]->track_id << endl;
		if (tracks[i]->movements.size() > 1 && tracks[i]->misses == 0)
		{
			int t_length = tracks[i]->movements.size();
			if (t_length > interval) { //--avg interval frames
				vector<float> speeds_x, speeds_y;
				for (int j = 0; j < t_length; j++) {
					speeds_x.push_back(tracks[i]->movements[j].x);
					speeds_y.push_back(tracks[i]->movements[j].y);
				}
				sort(speeds_x.begin(), speeds_x.end());
				sort(speeds_y.begin(), speeds_y.end());

				float total_x = 0, total_y = 0;
				int count = 0;
				for (int k = speeds_x.size() / 3; k < 2 * speeds_x.size() / 3; k++) {
					count++;
					total_x += speeds_x[k];
					total_y += speeds_y[k];
				}
				float s = sqrt(total_x*total_x+total_y*total_y) / count;
				//if (tracks[i]->track_id == 1) {
				//	rectangle(wrapped_frame, tracks[i]->box, Scalar(0, 255, 0), 2);
				//	circle(frame, tracks[i]->trace.back(), 3, Scalar(0, 0, 255), 2, FILLED);
				//	circle(frame, tracks[i]->trace[old_t], 3, Scalar(0, 255, 0), 2, FILLED);
				//	circle(wrapped_frame, warped_center, 3, Scalar(0, 0, 255), 2, FILLED);
				//	circle(wrapped_frame, warped_center_old, 3, Scalar(0, 255, 0), 2, FILLED);
				//}

				//-- convert to mph
				if (counter % update_freq == 0)
					tracks[i]->velocity = ftpp * s * 3600.0 * fps/5280.0;
				//cout << "counter: " << counter << " s: " << to_string(s) << "\tspeed: " << tracks[i]->speed << endl;

			}	
		}
	}
	counter = (counter + 1) % 1000;
}
