#pragma once
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cstdlib>

using namespace std;
using namespace cv;
class Image
{

private:

	Mat img;
	RNG rng;
	//vector<vector<Point>> contours;

	Vec3f findSlopeAndConstantB(const Point& point1, const Point& point2);
	vector<Vec4f> findLineIntersectionPoints(const vector<Vec3f>& lines);
	vector<Vec4f> assignQuadrant(vector<Vec4f>& points, const Vec2f& imageSize);
	vector<Vec4f> findFinalPoints(vector<Vec4f>& points);
	vector<Vec4f> roundAllPoints(vector<Vec4f>& points);
	vector<Vec4f> addMiddlePoint(vector<Vec4f>& points);
	float euclideanDist(const Vec4f& p1, const Vec4f& p2);
	Vec4f findAveragePoint(const vector<Vec4f>& points);
	float round(float& value);
public:

	Image() {};

	Mat loadImage(const string& s);
	void displayImage(const Mat& img1);
	Mat thresholdImage(Mat& img1, const int& thresholdValue, const int& maxValue);
	Mat findBiggestConnectedComponent( Mat& img1);
	Mat applyGaussianAndCannyEdge(Mat& img1);
	Mat applyHoughTransformAndFindPoints(Mat & img1);

	~Image() {};
};
