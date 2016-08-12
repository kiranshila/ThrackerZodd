#pragma once

#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

using namespace cv;

#include "Images.h"
#include "Points.h"
#include "Finder.h"
#include "Kalman.h"

//#define _windows_

FILE* openFile(const char* f) {
	FILE* out;
#ifdef _windows_
	fopen_s(&out, f, "r");
#else
	out = fopen(f, "r");
#endif
	return out;
}

class Tracker {
public:
	std::vector<Point3f> marker_corners;
	std::vector<point> marker_inside;
	Mat cameraMatrix, distCoeffs;
	Mat rvec;
	Mat tvec;
	vecFilter rvecf, tvecf;
	int width, height;
	VideoCapture cap;
	image I;
	std::vector<point> P;
	std::vector<std::vector<Point2f>> C;
	std::vector<std::vector<int>> N;
	bool trackerFound;
	Tracker() {
		trackerFound = false;
	}
	Tracker(char* config) {
		float fx = 1, fy = 1, cx = .5, cy = .1, d1 = 0, d2 = 0, d3 = 0, d4 = 0;
		FILE * file;
		trackerFound = false;
		file = openFile(config);
		std::vector<String> fname;
		if (file != NULL) {
			std::string tmp;
			int n = 0;
			while (n < 256)
			{
				char c = fgetc(file);
				if (feof(file))
				{
					break;
				}
				if (c == ' ') {
					fname.push_back(tmp);
					tmp = "";
				}
				else
				{
					tmp += c;
				}
				n++;
			}
			fname.push_back(tmp);
			FILE * file2, *file3;
			file2 = openFile(fname[0].c_str());
			if (file2 != NULL) {
				n = 0;
				std::string val;
				std::vector<float> param;
				while (n < 1024)
				{
					char c = fgetc(file2);
					if (feof(file2))
					{
						break;
					}
					if (c == ' ') {
						param.push_back(stof(val));
						val = "";
					}
					else
					{
						val += c;
					}
					n++;
				}
				param.push_back(stof(val));
				fx = param[0];
				cx = param[1];
				fy = param[2];
				cy = param[3];
				d1 = param[4];
				d2 = param[5];
				d3 = param[6];
				d4 = param[7];
				width = int(param[8]);
				height = int(param[9]);
			}
			fclose(file2);
			file3 = openFile(fname[1].c_str());
			if (file3 != NULL) {
				n = 0;
				std::string val;
				std::vector<float> param;
				while (n < 1024)
				{
					char c = fgetc(file3);
					if (feof(file3))
					{
						break;
					}
					if (c == ' ') {
						param.push_back(stof(val));
						val = "";
					}
					else
					{
						val += c;
					}
					n++;
				}
				param.push_back(stof(val));
				float side = param[0] / 2.0;
				marker_corners.push_back(Point3f(-side, -side, 0));
				marker_corners.push_back(Point3f(-side, side, 0));
				marker_corners.push_back(Point3f(side, side, 0));
				marker_corners.push_back(Point3f(side, -side, 0));
				for (int m = 0; m < int(0.5*(param.size() - 1)); m++) {
					marker_inside.push_back({ param[2 * m + 1], param[2 * m + 2], 0, 0 });
				}
			}
			fclose(file3);
		}
		fclose(file);

		cameraMatrix = (Mat1f(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

		distCoeffs = Mat(4, 1, cv::DataType<double>::type);
		distCoeffs.at<double>(0) = d1;
		distCoeffs.at<double>(1) = d2;
		distCoeffs.at<double>(2) = d3;
		distCoeffs.at<double>(3) = d4;

		cap.open(std::stoi(fname[2]));
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
		cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
		cap.set(CV_CAP_PROP_FPS, 30);

		rvec = Mat(1, 3, cv::DataType<double>::type);
		tvec = Mat(1, 3, cv::DataType<double>::type);

		rvecf = vecFilter(1.0, 64);
		tvecf = vecFilter(1.5, 128);

		rvec *= 0;
		tvec *= 0;

		I = image(width, height);
	}
	void update() {
		trackerFound = false;
		int attempts = 0;
		do {
			cap >> img;
			attempts++;
			waitKey(1);
		} while (img.empty() && attempts < 100);

		cvtColor(img, gray_image, CV_BGR2GRAY);

		I.data = gray_image.data;
		
		P.clear();
		N.clear();
		C.clear();
		findPoints(I, P, 250, 200, 0.2);
		findNabors(P, N, marker_inside.size() + 4);
		findMarkers(P, N, marker_inside, C, 0.001);

		if (C.size() > 0) {
			trackerFound = true;
			solvePnP(Mat(marker_corners), Mat(C[0]), cameraMatrix, distCoeffs, rvec, tvec);// , true, SOLVEPNP_P3P);
			rvecf.update((float)rvec.at<double>(0), (float)rvec.at<double>(1), (float)rvec.at<double>(2));
			tvecf.update((float)tvec.at<double>(0), (float)tvec.at<double>(1), (float)tvec.at<double>(2));
		}
	}
};