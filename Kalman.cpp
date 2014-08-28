//#pragma once
#include "Kalman.h"

MyKalmanFilter::MyKalmanFilter()
{
	kalmanFilter = new KalmanFilter(4,2,0);
	state.create(4,1);
	processNoise.create(4, 1, CV_32F);
	measurement.create(2,1);
	measurement.setTo(Scalar(0));    
}

MyKalmanFilter::~MyKalmanFilter()
{

}
void MyKalmanFilter::Configure(Point position)
{
	kalmanFilter->statePre.at<float>(0) = position.x;
	kalmanFilter->statePre.at<float>(1) = position.y;
	kalmanFilter->statePre.at<float>(2) = 0;
	kalmanFilter->statePre.at<float>(3) = 0;

	kalmanFilter->transitionMatrix = (Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);
	
    setIdentity(kalmanFilter->measurementMatrix);
    setIdentity(kalmanFilter->processNoiseCov, Scalar::all(1e-4));
    setIdentity(kalmanFilter->measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(kalmanFilter->errorCovPost, Scalar::all(.1));
}

Point MyKalmanFilter::Predict()
{
	Mat prediction = kalmanFilter->predict();
	predictPoint = Point2f(prediction.at<float>(0),prediction.at<float>(1)); 
	return predictPoint;
}

void MyKalmanFilter::GenerateMeasurement(Point position)
{
	measurement.at<float>(0) = position.x;
	measurement.at<float>(1) = position.y;
	measurement += kalmanFilter->measurementMatrix * state;
}
Point MyKalmanFilter::Correct()
{
	Mat estimated = kalmanFilter->correct(measurement);
	return Point(estimated.at<float>(0),estimated.at<float>(1));
}
void MyKalmanFilter::ProcessNoise()
{
	randn( processNoise, Scalar(0), Scalar::all(sqrt(kalmanFilter->processNoiseCov.at<float>(0, 0))));
    state = kalmanFilter->transitionMatrix * state + processNoise;	
}
Point MyKalmanFilter::Tracking(Point p)
{
	GenerateMeasurement(p);
	Point estimated = Correct();
	return estimated;
}

