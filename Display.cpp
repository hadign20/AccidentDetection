#include "include/Display.h"

void Display::showTracks(Mat& frame, vector<Track*>& tracks, const std::vector<std::string>& classNames) {
	for (int i = 0; i < tracks.size(); i++)
	{
		if (tracks[i]->trace.size() > 1 && tracks[i]->misses == 0)
		{
			int classId = tracks[i]->classId;
			int x = tracks[i]->box.x;
			int y = tracks[i]->box.y;

#if DRAW_BBOX
			cv::rectangle(frame, tracks[i]->box, Colors[classId], 2);
			//DrawFilledRect(frame, tracks[i]->box, Colors[classId], 50);
#endif

#if DRAW_TRAJECTORY
			if (tracks[i]->trace.size() > SMOOTH_SIZE)
				for (int j = 0; j < tracks[i]->trace.size() - SMOOTH_SIZE; j += SMOOTH_SIZE)
					line(frame, tracks[i]->trace[j], tracks[i]->trace[j + SMOOTH_SIZE], Colors[classId], 2);
#endif

#if DRAW_LABEL
			std::string label = "";
			//-- label with only class
			label = classNames[classId];

			////-- label with speed
			//if (tracks[i]->speed >= 3.0) {
			//	std::string speed = std::to_string((int)tracks[i]->speed);// +": " + std::to_string(track.m_confidence);
			//	label = classNames[classId] + " " + speed + "mph";
			//}
			//else
			//	label = classNames[classId];



			int baseline = 0;
			cv::Size size = cv::getTextSize(label, cv::FONT_ITALIC, 0.8, 2, &baseline);
			//DrawFilledRect(frame, cv::Rect(cv::Point(x, y - 25), cv::Point(x + size.width, y)), Scalar(Colors[classId][0] - 200, Colors[classId][1] - 200, Colors[classId][2] - 200), 200);
			cv::rectangle(frame, cv::Point(x, y - 25), cv::Point(x + size.width, y), Scalar(Colors[classId][0] - 200, Colors[classId][1] - 200, Colors[classId][2] - 200), -1);
			cv::putText(frame, label, cv::Point(x, y - 3), cv::FONT_ITALIC, 0.8, cv::Scalar(255, 255, 255), 2);
#endif

		}
	}
}



void Display::DrawFilledRect(cv::Mat& frame, const cv::Rect& rect, cv::Scalar cl, int alpha)
{
	if (alpha)
	{
		const int alpha_1 = 255 - alpha;
		const int nchans = frame.channels();
		int color[3] = { cv::saturate_cast<int>(cl[0]), cv::saturate_cast<int>(cl[1]), cv::saturate_cast<int>(cl[2]) };
		for (int y = rect.y; y < rect.y + rect.height; ++y)
		{
			if (y < 0) y = 0;
			if (y >= frame.rows) y = frame.rows - 1;
			uchar* ptr = frame.ptr(y) + nchans * rect.x;
			for (int x = rect.x; x < rect.x + rect.width; ++x)
			{
				if (x < 0) x = 0;
				if (x >= frame.cols) x = frame.cols - 1;
				for (int i = 0; i < nchans; ++i)
				{
					//cout << "x: " << x << " y: " << y << endl;
					ptr[i] = cv::saturate_cast<uchar>((alpha_1 * ptr[i] + alpha * color[i]) / 255);
				}
				ptr += nchans;
			}
		}
	}
	else
	{
		cv::rectangle(frame, rect, cl, cv::FILLED);
	}
}




void Display::DrawFilledCircle(cv::Mat& frame, const cv::Point center, const int radius, cv::Scalar cl, double alpha)
{
	if (alpha)
	{
		cv::Mat overlay;  // declaring overlay matrix, we'll copy source image to this matrix
		frame.copyTo(overlay);  // copying the source image to overlay matrix, we'll be drawing shapes on overlay matrix and we'll blend it with original image

		// change this section to draw the shapes you want to draw
		cv::circle(overlay, center, radius, cl, -1);  // drawing circles on overlay image

		cv::addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame);  // blending the overlay (with alpha opacity) with the source image (with 1-alpha opacity)
	}
	else
	{
		cv::circle(frame, center, radius, cl, 2);
	}
}