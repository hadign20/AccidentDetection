#define _CRT_SECURE_NO_WARNINGS


#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

#include <iostream>
#include <vector>
#include "string.h"
#include <time.h>

#include "include/yolo_utils.h"
#include "include/detector.h"
#include "include/Intersection.h"
#include "include/MultiCams.h"
#include "include/Multitracker.h"
#include "include/Display.h"
#include "include/SpeedEstimator.h"


using namespace cv;
using namespace std;
string getFileName(std::string filePath);
Point2f Point_warp(Mat Trans, Point p);
string getTimeString( int numFrame, int frameRate, int h = 0, int m = 0, int s = 0);
float overlapRate(cv::Rect box1, cv::Rect box2);

int main(int argc, char** argv)
{

	Mat frame, thresh_frame, image,mask;
	vector<Mat> icons;
	string timestamp;
	vector<Mat> channels;
	vector<Mat> rois;
	vector<int> car_size;
	int ave_car = 0;
	Scalar Colors[9] = { Scalar(255,0,0),Scalar(0,0,255),Scalar(0,255,0),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127) };

	double a[3][3] = { 1169.1,  0 , 959.5,
		0 , 1168.66,  539.5,
		0,  0 , 1 };
	
	/*{ 1142.64,0,959.5,
	0, 1152.77, 539.5,
	0, 0, 1
	};*/
	Mat cameraMatrix(3, 3, CV_64F,a);
	double b[5] = { -0.396267, 0.202849 ,0.00852173, - 0.00236845, - 0.0458549 };
	//{ -0.3901856772906178, 0.3242830096455762, 0.01207189508551394, 0.004078602731888629, -0.21419693163446 };
	Mat distCoeffs(1, 5, CV_64F,b);

	//configs
	const float confThreshold = 0.3f;
	const float iouThreshold = 0.4f;
	const float ftpp = 0.30;//us46 
		//0.3; //rt21_3rd ave
						//0.508;//us1 wynwood2 
					   //0.4411;// us1 promenade   
						// 0.446;	//us1 wynwood
	bool isGPU = 1;// cmd.exist("gpu");
	int front = 1;
	string resultPath = "./results/";
	std::string iconPath = "./icons/";
	std::string classNamesPath = "./data/traffic.names";// cmd.get<std::string>("class_names");
	std::string GISPath = "GIS_rt130_princeton.xml";// cmd.get<std::string>("image");
	std::string modelPath = "models/custom2_1280.onnx";// yolov7_5classes_640x640.onnx";// cmd.get<std::string>("model_path");
	std::string cameras_file = "cameras.xml";



	if (argc > 1) 
		GISPath = argv[1];
	if (argc > 2)
		modelPath = argv[2];
	if (argc > 3)
		cameras_file = argv[3];
	
	//cout << GISPath << endl;

	//load class names
	const std::vector<std::string> classNames = yolo_utils::loadNames(classNamesPath);
	if (classNames.empty())
	{
		std::cerr << "Error: Empty class names file." << std::endl;
		return -1;
	}

	//object detector
	YOLODetector detector{ nullptr };
	try
	{
		detector = YOLODetector(modelPath, isGPU, cv::Size(1280, 1280));
		std::cout << "Model was initialized." << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return -1;
	}


	//read video files and transmission matrices
	MultiCams* cameras = new MultiCams(cameras_file);
	vector<VideoCapture> cams;
	for (int i = 0; i < cameras->video_addresses.size(); i++) {
		VideoCapture cam;
		if (!cam.open(cameras->video_addresses[i])) {
			std::cout << "Error opening video stream or file " + to_string(i) << std::endl;
			return -1;
		}
		cams.push_back(cam);
	} 

	//load intersection information
	Intersection GIS_map;
	GIS_map.loadConfig(GISPath);


	//create tracks
	Multitracker* multiTracker = new Multitracker;

	//speed estimation
	SpeedEstimator* speedEstimator = new SpeedEstimator;

	//create display window
	
	Display* displayer = new Display;
	cv::namedWindow("output", WINDOW_NORMAL);
	cv::namedWindow("display", WINDOW_NORMAL);
	//cv::namedWindow("mask", WINDOW_NORMAL);

	//load icons
	for (int i = 0; i < classNames.size(); i++) {
		Mat icon = imread(iconPath + classNames[i] + ".jpg");
		resize(icon, icon, Size(100, 100));
		icons.push_back(icon);
		//imshow("icon"+to_string(i), icon);
	}


	//output video
	string videoName = getFileName(cameras->video_addresses[0]);
	int frameRate = cams[0].get(CAP_PROP_FPS);
	int width = cams[0].get(CAP_PROP_FRAME_WIDTH);
	int height = cams[0].get(CAP_PROP_FRAME_HEIGHT);
	VideoWriter outputVideo(resultPath+videoName + "_result.mp4", VideoWriter::fourcc('A', 'V', 'C', '1'), frameRate, cv::Size(width, height), true);
	VideoWriter outputVideo2(resultPath + videoName + "_map.mp4", VideoWriter::fourcc('A', 'V', 'C', '1'), frameRate, cv::Size(GIS_map.lane_map.cols, GIS_map.lane_map.rows), true);
	ofstream out(resultPath + videoName + ".txt");
	ofstream lane_change_file(resultPath + videoName + "_lane_change.txt");
	ofstream counting_file(resultPath + videoName + "_counting.txt");
	ofstream conflict_file(resultPath + videoName + "_conflict.txt");

	//set ROI mask
	for (int i = 0; i < cameras->video_addresses.size(); i++) {
		if (cameras->rois[i].size() < 3) {
			Mat mask(cv::Size(width, height), CV_8UC1, Scalar::all(255));
			rois.push_back(mask);
		}
		else {
			Mat mask(cv::Size(width, height), CV_8UC1, Scalar::all(0));
			const Point* pts = (const Point*)Mat(cameras->rois[i]).data;
			int npts = Mat(cameras->rois[i]).rows;
			fillConvexPoly(mask,
				pts,
				npts,
				255, 8, 0);
			rois.push_back(mask);
		}
	}

	int n_frame = 0;
	bool bSuccess = true;
	while ((char)cv::waitKey(1) != 'q' && bSuccess)
	{
		n_frame++;

		timestamp = getTimeString(n_frame, frameRate, cameras->hours[0], cameras->minutes[0], cameras->seconds[0]);


		//get local time

		/*time_t nowtime = time(NULL); 
		tm* t = localtime(&nowtime);
		timestamp = to_string(t->tm_year + 1900) + "-" + to_string(t->tm_mon + 1) + "-" + to_string(t->tm_mday )
			+ " " + to_string(t->tm_hour) + ":" + to_string(t->tm_min) + ":" + to_string(t->tm_sec);*/

		int cam_n = cams.size();

		//get the next frame for all cameras
		for (int c = 0; c < cam_n; c++) {
			cams[c].grab();
		}
		if (n_frame < 19000)
			continue;
		//objects in traffic
		std::vector<Detection> traffic_detections;

		//intersection bird eye view
		Mat display = GIS_map.bg.clone();// (Size(frame.cols, frame.rows), CV_8UC3);
		Mat original;
		cv::waitKey(1);

		//process all the cameras
		for (int c = 0; c < cam_n; c++) {
			//read frame
			bSuccess = cams[c].retrieve(original);
			if (!bSuccess)
			{
				cout << "Miss a frame from camera " << c << endl;
				break;
			}
			else {
				//cv::imshow("original", original);
				Mat Trans = cameras->tMatrix[c];
				undistort(original, frame, cameraMatrix, distCoeffs);
				//Mat warped;
				//cv::warpPerspective(frame, warped, Trans, Size(display.cols, display.rows), INTER_LINEAR);
				//cv::imshow("warped", warped);
				//detect foreground
				//cameras[c].MOG_FG->apply(frame, mask, 0.003);

				//detect objects
				std::vector<Detection> result;
				result = detector.detect(frame, confThreshold, iouThreshold);

				//draw the ROI
				polylines(frame, cameras->rois[c], 2, Scalar(0, 0, 255), 3, 8, 0);
				//imshow("roi", rois[c]);


				//convert each detected object to the birds eyeview
				for (size_t i = 0; i < result.size(); i++) {
					//if (result[i].classId < 4 || result[i].classId == 5 || result[i].classId == 7) {
					Point2f center = Point2f(result[i].box.x + result[i].box.width / 2, result[i].box.y + result[i].box.height / 2);
					
					//if the centroid is out of ROI, continue
					if (rois[c].at<uchar>(center.y, center.x) == 0)
						continue;


					Detection d;
					d.classId = result[i].classId;
					d.orig_box = result[i].box;
					d.front_point = cameras->front_point[c];

					//calculate the GPS coordinates



					//if the bounding box is close to the edge, discard it
					if (d.orig_box.x < 10 || d.orig_box.y < 10 || d.orig_box.x + d.orig_box.width > frame.cols - 10 || d.orig_box.y + d.orig_box.height > frame.rows - 10)
						continue;

					//warp the four corners of teh bounding box
					Point temp = Point(result[i].box.x, result[i].box.y);
					int x = INT_MAX, y = INT_MAX;
					temp = Point_warp(Trans, temp);
					x = min(x, temp.x);
					y = min(y, temp.y);
					if (temp.x <= 0 || temp.y <= 0 || temp.x >= display.cols || temp.y >= display.rows)
						continue;
					d.outline.push_back(temp);
					temp = Point(result[i].box.x, result[i].box.y + result[i].box.height);
					temp = Point_warp(Trans, temp);
					x = min(x, temp.x);
					y = min(y, temp.y);
					if (temp.x <= 0 || temp.y <= 0 || temp.x >= display.cols || temp.y >= display.rows)
						continue;
					d.outline.push_back(temp);
					temp = Point(result[i].box.x + result[i].box.width, result[i].box.y + result[i].box.height);
					temp = Point_warp(Trans, temp);
					x = min(x, temp.x);
					y = min(y, temp.y);
					if (temp.x <= 0 || temp.y <= 0 || temp.x >= display.cols || temp.y >= display.rows)
						continue;
					d.outline.push_back(temp);
					temp = Point(result[i].box.x + result[i].box.width, result[i].box.y);
					temp = Point_warp(Trans, temp);
					x = min(x, temp.x);
					y = min(y, temp.y);
					if (temp.x <= 0 || temp.y <= 0 || temp.x >= display.cols || temp.y >= display.rows)
						continue;
					d.outline.push_back(temp);
					center = Point_warp(Trans, center);
					if (center.x <= 0 || center.y <= 0 || center.x >= display.cols || center.y >= display.rows)
						continue;
					//if in lane, amend it to the center of the lane
					/*int current_lane = GIS_map.lane_map.at<Vec3b>(center.y, center.x)[0];
					if (current_lane != 0 && current_lane != 255) {
						vector<Point> t = GIS_map.approaches[current_lane / 10].trajectories[current_lane % 10 - 1];
						Point amend_center;
						amend_center.x = ((t[0].x * t[1].y - t[1].x * t[0].y - t[0].x * center.y + t[1].x * center.y) * (t[1].y - t[0].y) + center.x * (t[0].x - t[1].x) * (t[0].x - t[1].x))
							/ ((t[1].x - t[0].x) * (t[1].x - t[0].x) + (t[1].y - t[0].y) * (t[1].y - t[0].y) + 1);
						amend_center.y = (amend_center.x * (t[0].y - t[1].y) + t[0].x * t[1].y - t[1].x * t[0].y) / (t[0].x - t[1].x + 1);

						d.centroid = amend_center;
					}
					else*/
						d.centroid = center;


					int w = abs(2 * (center.x - x));
					int h = abs(2 * (center.y - y));
					d.box = Rect(x, y, w, h);

					//estimate the average size of the vehicle
					if (car_size.size() < 10000) {
						if (d.classId == 2) {
							int size = d.box.area();
							car_size.push_back(size);
						}
					}
					else if (car_size.size() == 10000) {
						sort(car_size.begin(), car_size.end());
						int total = 0;
						for (int s = 1000; s < 7000; s++) {
							total += car_size[s];
						}
						ave_car = total / 6000;
						//cout <<"Average:"<< ave_car << endl;
						car_size.push_back(0);
					}
					else {	//if the size of a box is too big, and classified as a car, change it to truck
						if (d.box.area() > 5 * ave_car && d.classId == 2) {
							//cout << d.classId << "," << d.box.area() << endl;
							d.classId = 4;
						}
						//else if (d.box.area() <= ave_car && d.box.area() > 0.25 * ave_car && (d.classId == 3 || d.classId == 4)) {
						//	cout << d.classId << "," << d.box.area() << endl;
						//	d.classId = 2;
						//}

					}

					// combine the close objects from multiple cameras
					bool found = false;
					for (int j = 0; j < traffic_detections.size(); j++) {
						if ((d.classId < 2) && traffic_detections[j].classId == 1) { // if a person riding a cyclist, discard the person
							//cout << overlapRate(d.orig_box, traffic_detections[j].orig_box) << endl;
							if (overlapRate(d.orig_box, traffic_detections[j].orig_box) > 0.3) {
								//cout << "riding detected" << endl;
								found = true;
								break;
							}
						}
						if (abs(traffic_detections[j].centroid.x - center.x) < 20 &&
							abs(traffic_detections[j].centroid.y - center.y) < 20) {
							found = true;
							break;
						}



					}
					//cv::rectangle(frame, result[i].box, Scalar(255, 255, 0), 2);

					//cout << found << "," << d.classId << endl;
					if (!found)
						traffic_detections.push_back(d);
					//}

				}

			}




		}
		//cout << traffic_detections.size() << endl;
		//for (int i = 0; i < traffic_detections.size(); i++) {
		//	//cout << traffic_detections[i].box << endl;
		//	polylines(display, traffic_detections[i].outline, 1, Scalar(255, 255, 0), 1, 8, 0);
		//	cv::rectangle(display, traffic_detections[i].box, Scalar(255,255,0), 2);

		//}
		if (!bSuccess)
			break;


		if (traffic_detections.size() >= 0)
		{
			multiTracker->update_tracks(traffic_detections, GIS_map,front); 		//track objects
			speedEstimator->detectSpeed(multiTracker->tracks, frameRate, GIS_map.ftpp); //detect speed
			//displayer->showTracks(frame, multiTracker->tracks, classNames);
			multiTracker->lane_change(timestamp,lane_change_file);	//detect lane change
			multiTracker->countV(timestamp, frame, counting_file, GIS_map);		//count 
			multiTracker->conflict_detection(timestamp, classNames, frame, conflict_file, display);		//conflict detection
		}

		out << "======================= frame: "<< n_frame << endl;
		out << "Time: " << timestamp << endl;
		//DrawFilledRect(frame, cv::Rect(cv::Point(0, 10), cv::Size(400, 50)), cv::Scalar(0, 0, 0), 150);
		cv::putText(frame, cameras->cam_info[0], cv::Point(30, 30), cv::FONT_ITALIC, 1, cv::Scalar(0, 0, 0), 5);
		cv::putText(frame, cameras->cam_info[0], cv::Point(30, 30), cv::FONT_ITALIC, 1, cv::Scalar(255, 255, 255), 2);
		cv::putText(frame, to_string(n_frame), cv::Point(frame.cols - 100, frame.rows - 30), cv::FONT_ITALIC, 0.8, cv::Scalar(0, 0, 0), 5);
		cv::putText(frame, to_string(n_frame), cv::Point(frame.cols - 100, frame.rows - 30), cv::FONT_ITALIC, 0.8, cv::Scalar(255, 255, 255), 2);
		cv::putText(display, to_string(n_frame), cv::Point(display.cols - 100, display.rows - 30), cv::FONT_ITALIC, 0.8, cv::Scalar(255, 255, 255), 2);
		cv::putText(frame, timestamp, cv::Point(frame.cols - 300, 30), cv::FONT_ITALIC, 0.8, cv::Scalar(255, 255, 255), 5);
		cv::putText(frame, timestamp, cv::Point(frame.cols - 300, 30), cv::FONT_ITALIC, 0.8, cv::Scalar(0, 0, 255), 2);
		cv::putText(display, timestamp, cv::Point(display.cols - 300, 30), cv::FONT_ITALIC, 0.8, cv::Scalar(0, 0, 0), 2);

		//for (int i = 0; i < GIS_map.approaches.size(); i++) {
		//	Approach ap = GIS_map.approaches[i];
		//	for (int j = 0; j < ap.trajectories.size(); j++) {
		//		vector<Point> t = ap.trajectories[j];
		//		for (int k = 0; k < t.size() - 1; k++) {
		//			line(display, t[k],t[k + 1], Scalar(0, 255, 255), 2);

		//		}
		//	}
		//}


		for (int i = 0; i < multiTracker->tracks.size(); i++)
		{
			//displayer->smoothenTrack(multiTracker->tracks[i]->trace);
			//cout << tracks[i]->track_id << endl;
			if (multiTracker->tracks[i]->trace.size() > 1 && multiTracker->tracks[i]->misses < 5 && multiTracker->tracks[i]->classId > 0)
				//&& multiTracker->tracks[i]->lane[multiTracker->tracks[i]->lane.size() - 1] != 0)
			{
				float time_to_stopbar = 0;
				std::string label = classNames[multiTracker->tracks[i]->classId];
				std::string velocity = std::to_string((int)multiTracker->tracks[i]->velocity);// +": " + std::to_string(track.m_confidence);

				//display on original frame
				cv::rectangle(frame, multiTracker->tracks[i]->orig_box, Colors[multiTracker->tracks[i]->classId], 2);


				string output_string = label;
				//output_string += " track:" + to_string(multiTracker->tracks[i]->track_id);

				if (multiTracker->tracks[i]->velocity > 5)
					output_string += ":" + velocity + "mph";
				if (multiTracker->tracks[i]->lane[multiTracker->tracks[i]->lane.size() - 1] != 0 && multiTracker->tracks[i]->lane[multiTracker->tracks[i]->lane.size() - 1] != 255)
					output_string += " in lane " + to_string(multiTracker->tracks[i]->lane[multiTracker->tracks[i]->lane.size() - 1]%10);
				cv::putText(frame, output_string, cv::Point(multiTracker->tracks[i]->orig_box.x, multiTracker->tracks[i]->orig_box.y - 3), cv::FONT_ITALIC, 0.8, cv::Scalar(255, 255, 255), 2);
				//cv::putText(frame, to_string(multiTracker->tracks[i]->trace[multiTracker->tracks[i]->trace.size() - 1].x)+","+
				//	to_string(multiTracker->tracks[i]->trace[multiTracker->tracks[i]->trace.size() - 1].y), cv::Point(multiTracker->tracks[i]->orig_box.x, multiTracker->tracks[i]->orig_box.y - 3), cv::FONT_ITALIC, 0.8, cv::Scalar(255, 255, 255), 2);



				float dis_to_stop = multiTracker->tracks[i]->dist2StopBar[multiTracker->tracks[i]->trace.size() - 1];
				int lane = multiTracker->tracks[i]->lane[multiTracker->tracks[i]->lane.size() - 1];
				if (multiTracker->tracks[i]->velocity != 0)
					time_to_stopbar = dis_to_stop / multiTracker->tracks[i]->velocity / 1.4667; //1.4667 mph to fps
				if (time_to_stopbar != 0) {
					//putText(display, to_string(time_to_stopbar).substr(0, 5), multiTracker->tracks[i]->trace[multiTracker->tracks[i]->trace.size() - 1], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
					out << label << " in approach " << lane / 10 << ", lane " << lane % 10 << ". Distance to stop bar: " << dis_to_stop << " ft, speed: "<< (int)multiTracker->tracks[i]->velocity <<"mph, time to stop bar: " << time_to_stopbar << " s. " << endl;


				}
				//polylines(display, multiTracker->tracks[i]->outline, 1, CV_RGB(255, 0, 0), 1, 8, 0);
				//cv::rectangle(display, multiTracker->tracks[i]->box, Scalar(255, 0, 0), 2);
				
				//draw object on GIS map
				cv::Rect roi_rect = cv::Rect(multiTracker->tracks[i]->trace[multiTracker->tracks[i]->trace.size() - 1].x - icons[0].cols/2,multiTracker->tracks[i]->trace[multiTracker->tracks[i]->trace.size() - 1].y - icons[0].cols / 2, icons[0].rows, icons[0].cols);
				Mat icon;
				if (multiTracker->tracks[i]->speed.x <= 0)
					flip(icons[multiTracker->tracks[i]->classId], icon, 1);
				else
					icon = icons[multiTracker->tracks[i]->classId];

				icon.copyTo(display(roi_rect), icon);
				
				//circle(display, multiTracker->tracks[i]->trace[multiTracker->tracks[i]->trace.size() - 1], 5, cv::Scalar(255, 255, 255), 2);
				//out << label << " outside lane." << endl;

				for (int j = 0; j < multiTracker->tracks[i]->trace.size() - 1; j++)
				{
					line(display, multiTracker->tracks[i]->trace[j], multiTracker->tracks[i]->trace[j + 1], Scalar(0, 255, 0), 2);
					//cout << tracks[i]->classId<<":"<< tracks[i]->trace[j] << endl;

				}
				//cout << endl;
				if (multiTracker->tracks[i]->lane_change) {
					circle(display, multiTracker->tracks[i]->trace[multiTracker->tracks[i]->trace.size() - 1], 80, cv::Scalar(246, 246, 100), 5);
					circle(frame, Point((multiTracker->tracks[i]->orig_box.x + multiTracker->tracks[i]->orig_box.width / 2), (multiTracker->tracks[i]->orig_box.y + multiTracker->tracks[i]->orig_box.height / 2)), max(multiTracker->tracks[i]->orig_box.width, multiTracker->tracks[i]->orig_box.height), cv::Scalar(246, 246, 100), 3);

				}



			}
			if (multiTracker->tracks[i]->future_trace.size() > 1 && multiTracker->tracks[i]->velocity > 5) {
				for (int j = 0; j < multiTracker->tracks[i]->future_trace.size() - 1; j++)
				{
					line(display, multiTracker->tracks[i]->future_trace[j], multiTracker->tracks[i]->future_trace[j + 1], Scalar(0, 0, 255), 2);
					//cout << tracks[i]->classId<<":"<< tracks[i]->trace[j] << endl;
				}
			}

		}

		imshow("display", display);

		imshow("output", frame);

		//imshow("mask", mask);

		outputVideo << frame;
		outputVideo2 << display;

		cv::waitKey(1);
	}
	outputVideo.release();
	outputVideo2.release();
	//destroyAllWindows();
	return 0;

}


std::string getFileName(std::string filePath)
{
	std::string rawname, seperator;
	if (filePath.find_last_of('\\') == string::npos)
		seperator = '/';
	else
		seperator = '\\';

	std::size_t dotPos = filePath.rfind('.');
	std::size_t sepPos = filePath.rfind(seperator);

	rawname = filePath.substr(sepPos + 1, dotPos - sepPos - 1);

	return rawname;
}

Point2f Point_warp(Mat Trans, Point p)

{

	cv::Mat_<double> src(3, 1);
	src(0, 0) = p.x;
	src(1, 0) = p.y;
	src(2, 0) = 1.0;
	Mat_<double> dst = Trans * src;
	for (int i = 0; i < dst.cols; i++) {
		dst(0, i) = dst(0, i) / dst(2, i);
		dst(1, i) = dst(1, i) / dst(2, i);
	}
	//if (dst(0, 0) < 0)
	//	dst(0, 0) = 0;
	//if (dst(1, 0) < 0)
	//	dst(1, 0) = 0;
	return Point2f(dst(0, 0), dst(1, 0));
}

string getTimeString(int numFrame, int frameRate, int h, int m, int s) {
	// Add result to file
	int totalSecs = (numFrame / frameRate);
	int hours = (totalSecs / 3600 + h) % 24;
	int minutes = (totalSecs % 3600) / 60 + m;
	int seconds = totalSecs % 60 + s;
	if (seconds >= 60) {
		minutes++;
		seconds -= 60;
	}
	if (minutes >= 60) {
		hours++;
		minutes -= 60;
	}
	int miliseconds = numFrame % frameRate * 1000 / frameRate;
	string timeString = format("%02d:%02d:%02d.%03d", hours, minutes, seconds, miliseconds);
	return timeString;
}

float overlapRate(cv::Rect box1, cv::Rect box2)
{
	int x1 = box1.x, y1 = box1.y, w1 = box1.width, h1 = box1.height;
	int x2 = box2.x, y2 = box2.y, w2 = box2.width, h2 = box2.height;

	int endx = max(x1 + w1, x2 + w2);
	int startx = min(x1, x2);
	int width = w1 + w2 - (endx - startx);  
	int endy = max(y1 + h1, y2 + h2);
	int starty = min(y1, y2);
	int height = h1 + h2 - (endy - starty);  
	if (width > 0 && height > 0) {
		int area = width * height;  
		int area1 = w1 * h1;
		int area2 = w2 * h2;
		float ratio = (float) max(area/(0.1+area1),area/ (0.1 + area2));
		return ratio;
	}
	else 
		return 0.0;

}