#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

class MyKalmanFilter
{
private:
	Mat_<float> state; /* (x, y, Vx, Vy) */
    Mat processNoise;
    Mat_<float> measurement;
    Point predictPoint;
public:
	KalmanFilter* kalmanFilter;
	MyKalmanFilter();
	~MyKalmanFilter();
	void Configure(Point p);
	Point Predict();
	void GenerateMeasurement(Point p);
	Point Correct();
	void ProcessNoise();
	Point Tracking(Point p);
	// KalmanFilter* kalman;
	// double deltatime; //приращение времени
	// Point2f LastResult;
	// TKalmanFilter(Point2f p,float dt=0.2,float Accel_noise_mag=0.5);
	// ~TKalmanFilter();
	// Point2f GetPrediction();
	// Point2f Update(Point2f p, bool DataCorrect);

};

