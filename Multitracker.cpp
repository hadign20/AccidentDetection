#include "include/Multitracker.h"

Multitracker::Multitracker() {

}

Multitracker::~Multitracker() {

}

void Multitracker::update_tracks(std::vector<Detection>& detections, Intersection inters, int front)
{
	// --------------------------------
	// If there is no tracks yet, then every point begins its own track.
	// -----------------------------------
	if (tracks.size() == 0)
	{
		// If no tracks yet
		for (int i = 0; i < detections.size(); i++)
		{
			Point2f center;
			switch (detections[i].front_point) {
			case 0:
				center = detections[i].centroid;

				break;
			case 1:
				center = Point(detections[i].outline[0].x, detections[i].outline[0].y);
				break;
			case 2:
				center = Point((detections[i].outline[0].x + detections[i].outline[1].x)/2, (detections[i].outline[0].y + detections[i].outline[1].y) / 2);
				break;
			case 3:
				center = Point(detections[i].outline[1].x, detections[i].outline[1].y);
				break;
			case 4:
				center = Point((detections[i].outline[1].x + detections[i].outline[2].x) / 2, (detections[i].outline[1].y + detections[i].outline[2].y) / 2);
				break;
			case 5:
				center = Point(detections[i].outline[2].x, detections[i].outline[2].y);
				break;
			case 6:
				center = Point((detections[i].outline[2].x + detections[i].outline[3].x) / 2, (detections[i].outline[2].y + detections[i].outline[3].y) / 2);

				break;
			case 7:
				center = Point(detections[i].outline[3].x, detections[i].outline[3].y);
				break;
			case 8:
				center = Point((detections[i].outline[0].x + detections[i].outline[3].x) / 2, (detections[i].outline[0].y + detections[i].outline[3].y) / 2);
				break;
			default:
				center = Point(detections[i].box.x + detections[i].box.width / 2, detections[i].box.y + detections[i].box.height / 2);
			}


			Track* tr = new Track(center, dt, acceleration, inters);
			tr->classId = detections[i].classId;
			tr->box = detections[i].box;
			tr->orig_box = detections[i].orig_box;
			tr->outline.assign(detections[i].outline.begin(), detections[i].outline.end());
			tracks.push_back(tr);
		}
	}


	int N = tracks.size();
	int M = detections.size();
	if (M == 0 && N == 0)
		return;


	vector< vector<double> > Cost(N, vector<double>(M));
	vector<int> assignment;

	double dist;
	for (int i = 0; i < tracks.size(); i++)
	{
		// Point2d prediction=tracks[i]->prediction;
		// cout << prediction << endl;
		for (int j = 0; j < detections.size(); j++)
		{
			Point2f center;
			switch (detections[j].front_point) {
			case 0:
				center = detections[j].centroid;
				break;
			case 1:
				center = Point(detections[j].outline[0].x, detections[j].outline[0].y);
				break;
			case 2:
				center = Point((detections[j].outline[0].x + detections[j].outline[1].x) / 2, (detections[j].outline[0].y + detections[j].outline[1].y) / 2);
				break;
			case 3:
				center = Point(detections[j].outline[1].x, detections[j].outline[1].y);
				break;
			case 4:
				center = Point((detections[j].outline[1].x + detections[j].outline[2].x) / 2, (detections[j].outline[1].y + detections[j].outline[2].y) / 2);
				break;
			case 5:
				center = Point(detections[j].outline[2].x, detections[j].outline[2].y);
				break;
			case 6:
				center = Point((detections[j].outline[2].x + detections[j].outline[3].x) / 2, (detections[j].outline[2].y + detections[j].outline[3].y) / 2);

				break;
			case 7:
				center = Point(detections[j].outline[3].x, detections[j].outline[3].y);
				break;
			case 8:
				center = Point((detections[j].outline[0].x + detections[j].outline[3].x) / 2, (detections[j].outline[0].y + detections[j].outline[3].y) / 2);
				break;
			default:
				center = Point(detections[j].box.x + detections[j].box.width / 2, detections[j].box.y + detections[j].box.height / 2);
			}

			Point2f diff = (tracks[i]->prediction - center);
			dist = diff.x * diff.x + diff.y * diff.y;
			Cost[i][j] = dist;
		}
	}

	AssignmentProblemSolver APS;
	APS.Solve(Cost, assignment, AssignmentProblemSolver::optimal);

	// -----------------------------------
	// clean assignment from pairs with large distance
	// -----------------------------------
	// Not assigned tracks
	vector<int> not_assigned_tracks;

	for (int i = 0; i < assignment.size(); i++)
	{
		if (assignment[i] != -1)
		{
			if (Cost[i][assignment[i]] > max_distance)
			{
				assignment[i] = -1;
				// Mark unassigned tracks, and increment skipped frames counter,
				// when skipped frames counter will be larger than threshold, track will be deleted.
				not_assigned_tracks.push_back(i);
			}
		}
		else
		{
			// If track have no assigned detect, then increment skipped frames counter.
			tracks[i]->misses++;
		}

	}

	// -----------------------------------
	// If track didn't get detects long time, remove it.
	// -----------------------------------
	for (int i = 0; i < tracks.size(); i++)
	{
		if (tracks[i]->misses > max_misses)
		{
			//delete tracks[i];
			tracks.erase(tracks.begin() + i);
			assignment.erase(assignment.begin() + i);
			i--;
		}
	}
	// -----------------------------------
	// Search for unassigned detects
	// -----------------------------------
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for (int i = 0; i < detections.size(); i++)
	{
		it = find(assignment.begin(), assignment.end(), i);
		if (it == assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}

	// -----------------------------------
	// and start new tracks for them.
	// -----------------------------------
	if (not_assigned_detections.size() != 0)
	{
		for (int i = 0; i < not_assigned_detections.size(); i++)
		{
			Point2f center;
			switch (detections[not_assigned_detections[i]].front_point) {
			case 0:
				center = detections[not_assigned_detections[i]].centroid;
				break;
			case 1:
				center = Point(detections[not_assigned_detections[i]].outline[0].x, detections[not_assigned_detections[i]].outline[0].y);
				break;
			case 2:
				center = Point((detections[not_assigned_detections[i]].outline[0].x + detections[not_assigned_detections[i]].outline[1].x) / 2, (detections[not_assigned_detections[i]].outline[0].y + detections[not_assigned_detections[i]].outline[1].y) / 2);
				break;
			case 3:
				center = Point(detections[not_assigned_detections[i]].outline[1].x, detections[not_assigned_detections[i]].outline[1].y);
				break;
			case 4:
				center = Point((detections[not_assigned_detections[i]].outline[1].x + detections[not_assigned_detections[i]].outline[2].x) / 2, (detections[not_assigned_detections[i]].outline[1].y + detections[not_assigned_detections[i]].outline[2].y) / 2);
				break;
			case 5:
				center = Point(detections[not_assigned_detections[i]].outline[2].x, detections[not_assigned_detections[i]].outline[2].y);
				break;
			case 6:
				center = Point((detections[not_assigned_detections[i]].outline[2].x + detections[not_assigned_detections[i]].outline[3].x) / 2, (detections[not_assigned_detections[i]].outline[2].y + detections[not_assigned_detections[i]].outline[3].y) / 2);

				break;
			case 7:
				center = Point(detections[not_assigned_detections[i]].outline[3].x, detections[not_assigned_detections[i]].outline[3].y);
				break;
			case 8:
				center = Point((detections[not_assigned_detections[i]].outline[0].x + detections[not_assigned_detections[i]].outline[3].x) / 2, (detections[not_assigned_detections[i]].outline[0].y + detections[not_assigned_detections[i]].outline[3].y) / 2);
				break;
			default:
				center = Point(detections[not_assigned_detections[i]].box.x + detections[not_assigned_detections[i]].box.width / 2, detections[not_assigned_detections[i]].box.y + detections[not_assigned_detections[i]].box.height / 2);
			}

			Track* tr = new Track(center, dt, acceleration, inters);
			tr->classId = detections[not_assigned_detections[i]].classId;
			tr->box = detections[not_assigned_detections[i]].box;
			tr->orig_box = detections[not_assigned_detections[i]].orig_box;
			tr->outline.assign(detections[i].outline.begin(), detections[i].outline.end());
			tracks.push_back(tr);
		}
	}

	// Update Kalman Filters state

	for (int i = 0; i < assignment.size(); i++)
	{
		// If track updated less than one time, than filter state is not correct.

		tracks[i]->KF->GetPrediction();

		if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
		{
			Point2f center;
			switch (detections[assignment[i]].front_point) {
			case 0:
				center = detections[assignment[i]].centroid;
				break;
			case 1:
				center = Point(detections[assignment[i]].outline[0].x, detections[assignment[i]].outline[0].y);
				break;
			case 2:
				center = Point((detections[assignment[i]].outline[0].x + detections[assignment[i]].outline[1].x) / 2, (detections[assignment[i]].outline[0].y + detections[assignment[i]].outline[1].y) / 2);
				break;
			case 3:
				center = Point(detections[assignment[i]].outline[1].x, detections[assignment[i]].outline[1].y);
				break;
			case 4:
				center = Point((detections[assignment[i]].outline[1].x + detections[assignment[i]].outline[2].x) / 2, (detections[assignment[i]].outline[1].y + detections[assignment[i]].outline[2].y) / 2);
				break;
			case 5:
				center = Point(detections[assignment[i]].outline[2].x, detections[assignment[i]].outline[2].y);
				break;
			case 6:
				center = Point((detections[assignment[i]].outline[2].x + detections[assignment[i]].outline[3].x) / 2, (detections[assignment[i]].outline[2].y + detections[assignment[i]].outline[3].y) / 2);

				break;
			case 7:
				center = Point(detections[assignment[i]].outline[3].x, detections[assignment[i]].outline[3].y);
				break;
			case 8:
				center = Point((detections[assignment[i]].outline[0].x + detections[assignment[i]].outline[3].x) / 2, (detections[assignment[i]].outline[0].y + detections[assignment[i]].outline[3].y) / 2);
				break;
			default:
				center = Point(detections[assignment[i]].box.x + detections[assignment[i]].box.width / 2, detections[assignment[i]].box.y + detections[assignment[i]].box.height / 2);
			}
			tracks[i]->misses = 0;
			tracks[i]->prediction = tracks[i]->KF->Update(center, 1);
			tracks[i]->UpdateTrack(center, detections[assignment[i]].box,inters);
			tracks[i]->orig_box = detections[assignment[i]].orig_box;
			tracks[i]->outline.assign(detections[assignment[i]].outline.begin(), detections[assignment[i]].outline.end());


		}
		else				  // if not continue using predictions
		{
			tracks[i]->prediction = tracks[i]->KF->Update(Point2f(0, 0), 0);
			tracks[i]->UpdateTrack(tracks[i]->prediction, tracks[i]->box, inters);

		}

		if (tracks[i]->trace.size() > max_trace)
		{
			tracks[i]->trace.erase(tracks[i]->trace.begin(), tracks[i]->trace.end() - max_trace);
			tracks[i]->lane.erase(tracks[i]->lane.begin(), tracks[i]->lane.end() - max_trace);
			tracks[i]->dist2StopBar.erase(tracks[i]->dist2StopBar.begin(), tracks[i]->dist2StopBar.end() - max_trace);
			tracks[i]->movements.erase(tracks[i]->movements.begin(), tracks[i]->movements.end() - max_trace);
		}

		//tracks[i]->KF->LastResult = tracks[i]->prediction;
	}
}

void Multitracker::lane_change(string timestamp, ofstream& file) {
	for (int i = 0; i < tracks.size(); i++)
	{
		int size = tracks[i]->lane.size();
		if (size >= 10 && tracks[i]->misses == 0)
		{
			if(tracks[i]->lane[size - 1] == tracks[i]->lane[0] || tracks[i]->velocity < 5)
				tracks[i]->lane_change = false;
			else {
				int pre_lane = tracks[i]->lane[0];
				int pre_time = 0;
				int current_lane = tracks[i]->lane[size - 1];
				int current_time = 0;
				int j = 0;
				if(pre_lane == 0 || pre_lane == 255 || current_lane == 0 || current_lane == 255)
					tracks[i]->lane_change = false;
				else {
					while (j < size && tracks[i]->lane[j] == pre_lane && pre_time < 5) {
						pre_time++;
						j++;
					}
					j = size - 1;
					while (j >= 0 && tracks[i]->lane[j] == current_lane && current_time < 5) {
						current_time++;
						j--;
					}
					if (pre_time == current_time && pre_time >= 5) {
						tracks[i]->lane_change = true;
						file << timestamp << " =======================" << endl;
						file << "change from lane " << pre_lane%10 << " to lane " << current_lane%10 << endl;
					}
				}
			}


			/*if (tracks[i]->lane[size - 1] == tracks[i]->lane[size - 15] && tracks[i]->lane[size - 1] != 0 && tracks[i]->lane[size - 1] != 255
				&& tracks[i]->lane[size - 16] == tracks[i]->lane[size - 30] && tracks[i]->lane[size - 16] != 0 && tracks[i]->lane[size - 16] != 255
				&& tracks[i]->lane[size - 1] != tracks[i]->lane[size - 16]) {
				tracks[i]->lane_change = true;
				file << "=======================" << n_frame << endl;
				file << "change from lane " << tracks[i]->lane[size - 16] << " to lane " << tracks[i]->lane[size - 1] << endl;
			}
			else
				tracks[i]->lane_change = false;*/
		}

	}
}

void Multitracker::countV(string timestamp, Mat& frame, ofstream& file, Intersection& GIS_map) {

	//clean the number of vehicles in current frame
	for (int i = 0; i < GIS_map.approaches.size(); i++) {
		Approach& ap = GIS_map.approaches[i];
		//cout << "Approach: " << i << endl;
		for (int j = 0; j < ap.current_v.size(); j++) {
			ap.current_v[j] = 0;
			//cout << "    Lane " << j << " current vehicle: " << ap.current_v[j] << ". Total vehecle: " << ap.total_v[j] << endl;

		}
	}
	for (int i = 0; i < tracks.size(); i++)
	{
		int size = tracks[i]->lane.size();
		int current_lane = tracks[i]->lane[size - 1];

		if (tracks[i]->misses == 0) {
			//cout << "before lane " << current_lane % 10 << " current vehicle: " << GIS_map.approaches[current_lane / 10].current_v[current_lane % 10] << ". Total vehecle: " << GIS_map.approaches[current_lane / 10].total_v[current_lane % 10] << endl;
			GIS_map.approaches[current_lane / 10].current_v[current_lane % 10 - 1]++;
			//cout << "after     Lane " << current_lane % 10 << " current vehicle: " << GIS_map.approaches[current_lane / 10].current_v[current_lane % 10] << ". Total vehecle: " << GIS_map.approaches[current_lane / 10].total_v[current_lane % 10] << endl;

			if (size == 15) {
				GIS_map.approaches[current_lane / 10].total_v[current_lane % 10 - 1]++;
				cv::rectangle(frame, tracks[i]->orig_box, Scalar(0, 25, 50), 16);

			}
		}
	}

	//output the counting
	file << endl;
	file << "=======================" << timestamp << endl;
	for (int i = 0; i < GIS_map.approaches.size(); i++) {
		file << "Approach " << i << endl;
		Approach ap = GIS_map.approaches[i];
		for (int j = 0; j < ap.current_v.size(); j++) {
			file << "    Lane " << j + 1 << " current vehicle: "<< ap.current_v[j] << ". Total vehecle: " << ap.total_v[j] << endl;
		}
	}

}

void Multitracker::conflict_detection(string timestamp, vector<std::string> classNames, Mat& frame, ofstream& file, Mat& GIS) {
	for (int i = 0; i < tracks.size(); i++)
	{
		Track* t1 = tracks[i];
		if (t1->classId > 1 && t1->velocity < 5 || t1->trace.size() < 1)
			continue;
		for (int j = i + 1; j < tracks.size(); j++) {
			Track* t2 = tracks[j];
			if (t2->classId > 1 && t2->velocity < 5 || t2->trace.size() < 1 || t1->classId == 0 && t2->classId == 0)
				continue;

			//trajectory conflicts
			int dis = (t1->trace[t1->trace.size() - 1].x - t2->trace[t2->trace.size() - 1].x) * (t1->trace[t1->trace.size() - 1].x - t2->trace[t2->trace.size() - 1].x)
				+ (t1->trace[t1->trace.size() - 1].y - t2->trace[t2->trace.size() - 1].y) * (t1->trace[t1->trace.size() - 1].y - t2->trace[t2->trace.size() - 1].y);
			float dir = abs(atan2(t1->speed.y,t1->speed.x) - atan2(t2->speed.y, t2->speed.x))*180/3.14;


			//cout << t1->classId<<","<< t2->classId<<":"<< dir <<",dis:"<<dis<< endl;
			//cout << atan2(t1->speed.y, t1->speed.x) * 180 / 3.14 << endl;
			//cout << atan2(t2->speed.y, t2->speed.x) * 180 / 3.14 << endl;
			//real conflict
			if (dis < 900 && dir > 30) {
				circle(GIS, t1->trace[t1->trace.size() - 1], 50, cv::Scalar(0, 0, 255), 3);
				circle(frame, Point((t1->orig_box.x+t1->orig_box.width/2), (t1->orig_box.y + t1->orig_box.height / 2)), 50, cv::Scalar(0, 0, 255), 3);
				circle(GIS, t2->trace[t2->trace.size() - 1], 50, cv::Scalar(0, 0, 255), 3);
				circle(frame, Point((t2->orig_box.x + t2->orig_box.width / 2), (t2->orig_box.y + t2->orig_box.height / 2)), 50, cv::Scalar(0, 0, 255), 3);

				file << timestamp << ":trajectory conflict between " << classNames[t1->classId] << " and " << classNames[t2->classId] << endl;
			}

			//potential conflict
			if (dir > 30) {
				int size = min(t1->future_trace.size(), t2->future_trace.size());
				for (int k = 0; k < size; k++) {
					dis = (t1->future_trace[k].x - t2->future_trace[k].x) * (t1->future_trace[k].x - t2->future_trace[k].x)
						+ (t1->future_trace[k].y - t2->future_trace[k].y) * (t1->future_trace[k].y - t2->future_trace[k].y);
					if (dis < 1600) {
						circle(GIS, t1->trace[t1->trace.size() - 1], 50, cv::Scalar(0, 0, 255), 3);						
						circle(GIS, t2->trace[t2->trace.size() - 1], 50, cv::Scalar(0, 0, 255), 3);
						circle(frame, Point((t1->orig_box.x + t1->orig_box.width / 2), (t1->orig_box.y + t1->orig_box.height / 2)), max(t1->orig_box.width, t1->orig_box.height), cv::Scalar(0, 0, 255), 3);
						circle(frame, Point((t2->orig_box.x + t2->orig_box.width / 2), (t2->orig_box.y + t2->orig_box.height / 2)), max(t2->orig_box.width, t2->orig_box.height), cv::Scalar(0, 0, 255), 3);

						file << timestamp << ":trajectory conflict between "<< classNames[t1->classId] << " and " << classNames[t2->classId] << endl;
					}
				}
			}

			//rear end conflicts
			if (t1->lane[t1->lane.size() - 1] == t2->lane[t2->lane.size() - 1]) {
				float d1 = t1->dist2StopBar[t1->dist2StopBar.size() - 1];
				float d2 = t2->dist2StopBar[t2->dist2StopBar.size() - 1];
				float v1 = t1->velocity;
				float v2 = t2->velocity;
				//cout << d1 << "," << d2 << ",,," << d1 - 2 * (v1 - v2) * 1.467 - d2 << endl;
				if (d1 > d2 && d1 - 2 * (v1 - v2) * 1.467 - d2 < 0) {
					circle(GIS, t1->trace[t1->trace.size() - 1], 50, cv::Scalar(0, 0, 255), 3);
					circle(frame, Point((t1->orig_box.x + t1->orig_box.width / 2), (t1->orig_box.y + t1->orig_box.height / 2)), max(t1->orig_box.width, t1->orig_box.height), cv::Scalar(0, 0, 255), 3);

					file << timestamp << ":rear end conflict in lane " << to_string(t1->lane[t1->lane.size() - 1]%10) << " between " << classNames[t1->classId] << " and " << classNames[t2->classId] << endl;
				}
				if (d2 > d1 && d2 - 2 * (v2 - v1) * 1.467 - d1 < 0) {
					circle(GIS, t2->trace[t2->trace.size() - 1], 50, cv::Scalar(0, 0, 255), 3);
					circle(frame, Point((t2->orig_box.x + t2->orig_box.width / 2), (t2->orig_box.y + t2->orig_box.height / 2)), max(t2->orig_box.width, t2->orig_box.height), cv::Scalar(0, 0, 255), 3);

					file << timestamp << ":rear end conflict in lane " << to_string(t1->lane[t1->lane.size() - 1]%10) << " between "<< classNames[t1->classId] << " and " << classNames[t2->classId] << endl;
				}
			}


		}
	}

}