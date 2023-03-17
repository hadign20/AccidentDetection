#include "include/MultiCams.h"

MultiCams::MultiCams(string configPath) {
	FileStorage fs(configPath, FileStorage::READ);
	int i = 0;
	while (!fs["camera" + to_string(i)].isNone()) {
		string videoAddress;
		int fp = 0;

		//read video addresses
		fs["camera" + to_string(i)] >> videoAddress;
		//cout << videoAddress << endl;
		video_addresses.push_back(videoAddress);

		//read video start time
		int h = 0, m = 0, s = 0;
		if (!fs["camera" + to_string(i) + "_h"].isNone())
			fs["camera" + to_string(i) + "_h"] >> h;
		hours.push_back(h);
		if (!fs["camera" + to_string(i) + "_m"].isNone())
			fs["camera" + to_string(i) + "_m"] >> m;
		minutes.push_back(m);
		if (!fs["camera" + to_string(i) + "_s"].isNone())
			fs["camera" + to_string(i) + "_s"] >> s;
		seconds.push_back(s);


		//read front points for each video,defalt '0' is the centroid
		if (!fs["camera" + to_string(i) + "_front"].isNone())
			fs["camera" + to_string(i) + "_front"] >> fp;
		front_point.push_back(fp);

		//read camera info
		string info;
		if (!fs["camera" + to_string(i) + "_info"].isNone())
			fs["camera" + to_string(i) + "_info"] >> info;
		cam_info.push_back(info);

		//read transmission matrices
		string transMFile;
		fs["matrix" + to_string(i)] >> transMFile;
		Mat Trans;
		FileStorage fs2(transMFile, FileStorage::READ);
		fs2["mat"] >> Trans;
		tMatrix.push_back(Trans);

		//read the ROI
		vector<Point> roi;
		if (!fs["camera" + to_string(i) + "_roi"].isNone()) {
			string roi_address;
			fs["camera" + to_string(i) + "_roi"] >> roi_address;
			FileStorage fs3(roi_address, FileStorage::READ);

			int x,y,k = 0;
			while (!fs3["roi_x" + to_string(k)].isNone()) {
				fs3["roi_x" + to_string(k)] >> x;
				fs3["roi_y" + to_string(k)] >> y;
				roi.push_back(Point(x, y));
				k++;
			}
		}
		rois.push_back(roi);


		i++;
	}
	return;
}
MultiCams::~MultiCams() {

}

