#include <stdio.h>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <fstream>

#include <time.h>

using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

using namespace cv;

#define _draw_
//#define _windows_

#include "Tracker.h"

int width = 640;// 1280;//1024;// 
int height = 480;// 720;//576;//

int main() {
	float fx = 1, fy = 1, cx = .5, cy = .1, d1 = 0, d2 = 0, d3 = 0, d4 = 0;
	vector<Point3f> marker_corners;
	vector<point> marker_inside;

	FILE * file;
#ifdef _windows_
	fopen_s(&file, "config.tz", "r");
#else
	file = fopen("config.tz", "r");
#endif
	if (file != NULL) {
		vector<String> fname;
		String tmp;
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
#ifdef _windows_
		fopen_s(&file2, fname[0].c_str(), "r");
#else
		file2 = fopen(fname[0].c_str(), "r");
#endif
		if (file2 != NULL) {
			n = 0;
			string val;
			vector<float> param;
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
#ifdef _windows_
		fopen_s(&file3, fname[1].c_str(), "r");
#else
		file3 = fopen(fname[1].c_str(), "r");
#endif
		if (file3 != NULL) {
			n = 0;
			string val;
			vector<float> param;
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

	VideoCapture cap;
	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
	cap.set(CV_CAP_PROP_FPS, 30);
	//cap.set(CV_CAP_PROP_EXPOSURE, 0);

	Mat cameraMatrix = (Mat1f(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

	Mat distCoeffs(4, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = d1;
	distCoeffs.at<double>(1) = d2;
	distCoeffs.at<double>(2) = d3;
	distCoeffs.at<double>(3) = d4;

#ifdef _draw_
	namedWindow("window", 1);
#endif

	time_t start, end;

	double sec = 0.05, counter = 1;

	image I = image(width, height);

	Mat rvec(1, 3, cv::DataType<double>::type);
	Mat tvec(1, 3, cv::DataType<double>::type);

	vecFilter rvecf(1.0, 64), tvecf(1.5, 128);

	rvec *= 0;
	tvec *= 0;

	while (1) {
		time(&start);

		int attempts = 0;
		do {
			cap >> img;
			attempts++;
			usleep(1000);
		} while (img.empty() && attempts < 100);

		cvtColor(img, gray_image, CV_BGR2GRAY);

		I.data = gray_image.data;

		vector<point> P;
		vector<vector<Point2f>> corners;
		vector<vector<int>> N;

		findPoints(I, P, 250, 200, 0.2);
		findNabors(P, N, marker_inside.size() + 4);
		findMarkers(P, N, marker_inside, corners, 0.001);

		if (corners.size() > 0) {
			cout << corners.size() << "\n";
			solvePnP(Mat(marker_corners), Mat(corners[0]), cameraMatrix, distCoeffs, rvec, tvec);// , true, SOLVEPNP_P3P);
			rvecf.update((float)rvec.at<double>(0), (float)rvec.at<double>(1), (float)rvec.at<double>(2));
			tvecf.update((float)tvec.at<double>(0), (float)tvec.at<double>(1), (float)tvec.at<double>(2));
		}

#ifdef _draw_
		if (corners.size() > 0) {
			line(img, corners[0][0], corners[0][1], Scalar(0, 0, 255));
			line(img, corners[0][1], corners[0][2], Scalar(0, 255, 0));
			line(img, corners[0][2], corners[0][3], Scalar(255, 0, 0));
			line(img, corners[0][3], corners[0][0], Scalar(128, 128, 128));
			circle(img, corners[0][0], 40, Scalar(0, 0, 255), 2, 8);
			circle(img, corners[0][1], 40, Scalar(0, 0, 255), 2, 8);
			circle(img, corners[0][2], 40, Scalar(0, 0, 255), 2, 8);
			circle(img, corners[0][3], 40, Scalar(0, 0, 255), 2, 8);
		}

		for (int n = 0; n < P.size(); n++) {
			circle(img, Point(P[n].x, P[n].y), sqrt(P[n].num / PI), Scalar(0, 255, 0), 4, 8);
		}

		imshow("window", img);
#endif

		time(&end);
		sec += difftime(end, start);
		counter++;
		sec *= 0.99;
		counter *= 0.99;
		double fps = counter / sec;
		std::cout << tvecf.print() << "\n" << tvec << "\n";
	}
	return 0;
}
