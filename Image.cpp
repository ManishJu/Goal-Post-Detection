#include "stdafx.h"
#include "Image.h"
#include <cstdlib>

using namespace std;




float Image::round(float & value)
{
	float result = (int)(value * 100 + .5);
	return (float)result / 100;
}

Mat Image::loadImage(const string & s)
{
	 Mat img;
	 img = imread(s);
	 return img; 
}

void Image::displayImage(const Mat& img) {

	
		namedWindow("image", WINDOW_AUTOSIZE);
		imshow("image", img);
		waitKey(0);
}


Mat Image::thresholdImage( Mat& img, const int& thresholdValue, const int& maxValue) {

		cvtColor(img, img, CV_RGB2GRAY);
		threshold(img, img, thresholdValue, maxValue, THRESH_BINARY);
		return img;
	
}

Mat Image::findBiggestConnectedComponent( Mat& img)
{

	Mat stats, centroids, cCImage;
	int nLabels = connectedComponentsWithStats(img, cCImage, stats, centroids, 4, CV_32S);
	Mat mask(cCImage.size(), CV_8UC1, Scalar(0));
	Mat surfSup = stats.col(4)>2000;

	for (int i = 1; i < nLabels; i++)
	{
		if (surfSup.at<uchar>(i, 0))
		{
			mask = mask | (cCImage == i);
		}
	}
	Mat temp(img.size(), CV_8UC1, Scalar(0));
	img.copyTo(temp, mask);
	
	img = temp;
	return img;


}

Mat Image::applyGaussianAndCannyEdge(Mat & img)
{
	
	Size ksize = { 9,9 };
	GaussianBlur(img, img, ksize, 7, 7);
	Canny(img, img, 200, 200, 3, 7);
	return img;
}

Mat Image::applyHoughTransformAndFindPoints(Mat & img)
{
	Mat  resultImage;

	cvtColor(img, resultImage, CV_GRAY2BGR);

	vector<Vec4i> lines;
	vector<Vec3f> slopeAndConstants;
	//Detecting hough lines in the target image;
	HoughLinesP(img, lines, 1, CV_PI / 360, 200, 400, 500);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		Point point1 = Point(l[0], l[1]);
		Point point2 = Point(l[2], l[3]);
		auto rscb = findSlopeAndConstantB(point1, point2);
		slopeAndConstants.push_back(rscb);
		line(resultImage, point1, point2, Scalar(0, 0, 255), 2, CV_AA);
		

	}

	Vec2f imgSize = { (float)resultImage.size().width,(float)resultImage.size().height };

	//Finding the intersection between lines
	vector<Vec4f> intersectionPoints = findLineIntersectionPoints(slopeAndConstants);
	//Assigning the points 4 quadrants(areas) in order of their location. Top left is quadrant 1
	//Top right is quadrant 2
	//Bottom left is quadrant 3
	//Bottom right is quadrant 4
	intersectionPoints = assignQuadrant(intersectionPoints,imgSize);
	//Averaging out the intersection points to find one concrete point
	intersectionPoints = findFinalPoints(intersectionPoints);
	//Rounding all the positions to 2 decimals of accuracy 
	intersectionPoints = roundAllPoints(intersectionPoints);
	//Keeping only unique positions of the goalPost 
	intersectionPoints.erase(unique(intersectionPoints.begin(), intersectionPoints.end()), intersectionPoints.end());
	//the middle point on the top between the 2 goalposts
	intersectionPoints = addMiddlePoint(intersectionPoints);

	//Adding goalpost point to the image
	for (unsigned int i = 0; i < intersectionPoints.size(); i++) {
			Point2f p = { intersectionPoints[i][0], intersectionPoints[i][1] };
			circle(resultImage, p, 5, Scalar(0, 255, 0), 2, 8, 0);	
	}
	std::ofstream f("algorithmGoalPosition0001.txt");
	for (unsigned int i = 0; i < intersectionPoints.size(); i++) {
		f << to_string(intersectionPoints[i][0])<<" "<<to_string(intersectionPoints[i][1]) << '\n';
	}
	// return the final image
	return resultImage;
	
}

Vec3f Image::findSlopeAndConstantB(const Point & point1, const Point & point2)
{
	float lineVertical = 1;
	Point diffP = point1 - point2;
	float slope = (float)diffP.y / (float)diffP.x;
	float constantB = point2.y - slope*point2.x;
	if (abs(slope) < 0.1) lineVertical = 0;
	Vec3f data = { slope,constantB,lineVertical };
	return data;
}

vector<Vec4f> Image::findLineIntersectionPoints(const vector<Vec3f>& lines)
{
	vector<Vec4f> points;
	Vec3f l, l2;
	float xPoint, yPoint;
	Vec4f point;
	for (unsigned int i = 0; i < lines.size(); i++) {
		l = lines[i];
		if (!l[2]) {
			for (unsigned int j = 0; j < lines.size(); j++) {
				l2 = lines[j];
				if (abs(l(2)) != abs(l2(2))) {
					xPoint = -(l[1] - l2[1]) / (l[0] - l2[0]);
					yPoint = l(0)*xPoint + l(1);
					point = { xPoint,yPoint,0,1 };
					points.push_back(point);
				}
			}
		}

	}
	return points;
}

vector<Vec4f> Image::assignQuadrant(vector<Vec4f>& points, const Vec2f & imageSize)
{
	Vec4f p;
	for (int i = 0; i < points.size(); i++) {
		p = points[i];
		if ((p[0] < imageSize[0] / 2) && (p[1] < imageSize[1] / 2)) p[2] = 1;
		else if ((p[0] < imageSize[0] / 2) && (p[1] > imageSize[1] / 2)) p[2] = 3;
		else if ((p[0] > imageSize[0] / 2) && (p[1] < imageSize[1] / 2)) p[2] = 2;
		else p[2] = 4;
		points[i][2] = p[2];


	}
	return points;
}

vector<Vec4f> Image::findFinalPoints(vector<Vec4f>& points)
{
	vector<Vec4f> intermediatePoints;
	vector<Vec4f> finalPoints;
	for (unsigned int i = 0; i < points.size(); i++) {
		auto p = points[i];
		intermediatePoints.clear();
		for (unsigned int j = 0; j < points.size(); j++) {
			auto p2 = points[j];
			auto dist = euclideanDist(p, p2);
			if ((p[2] == p2[2]) && dist < 70 ) {
				intermediatePoints.push_back(p2);
			}	
		}
		
		finalPoints.push_back(findAveragePoint(intermediatePoints));
	}
	return finalPoints;
}

vector<Vec4f> Image::roundAllPoints(vector<Vec4f>& points)
{
	for (unsigned int i = 0; i < points.size(); i++) {
		points[i][0] = round(points[i][0]);
		points[i][1] = round(points[i][1]);
		points[i][2] = round(points[i][2]);
		points[i][3] = round(points[i][3]);
	}
	return points;
}

vector<Vec4f> Image::addMiddlePoint(vector<Vec4f>& points)
{
	Vec4f topLeft, topRight;
	for (int i = 0; i < points.size(); i++) {
		if (points[i][2] == 1) {
			topLeft = points[i];
			break;
		}
	}
	for (int i = 0; i < points.size(); i++) {
		if (points[i][2] == 2) {
			topRight = points[i];
			break;
		}
	}

	for (int i = 0; i < points.size(); i++) {

		//top Left
		if ((points[i][2] == 1) && (topLeft[1] > points[i][1])) {
			topLeft = points[i];
			
		}
		//top Right
		if ((points[i][2] == 2) && (topRight[1] > points[i][1])) {
			topRight = points[i];

		}
	}

	//middle
	vector<Vec4f> Pts;
	Pts.push_back(topLeft);
	Pts.push_back(topRight);
	points.push_back(findAveragePoint(Pts));
	return points;

}

float Image::euclideanDist(const Vec4f & p1, const Vec4f & p2)
{
	Vec4f diff = p1 - p2;

	return sqrt(diff[0] * diff[0] + diff[1] * diff[1]);
}

Vec4f Image::findAveragePoint(const vector<Vec4f>& points)
{
	Vec4f avgPoint;
	if (points.size()) {
		float xValue = 0, yValue = 0;
		for (unsigned int i = 0; i < points.size(); i++) {
			xValue += points[i][0];
			yValue += points[i][1];
		}

		xValue /= points.size();
		yValue /= points.size();

		avgPoint = { xValue,yValue,points[0][2],points[0][3] };
	}

	return  avgPoint;
}
