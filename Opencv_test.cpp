﻿// Opencv_test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "Image.h"
#include <cstdlib>

using namespace std;
using namespace cv;


int main(int argc, char** argv)
{
	
	Image image1;

	string filename = "im0001.png";

	Mat resultImage = image1.loadImage(filename);

	//segmenting the image based on threshold
	//K-Means Clustering was also tested but failed as it mixed many prominent feature of the goalpost with the ropes
	resultImage = image1.thresholdImage(resultImage, 110, 255);
	//After thresholding the connected components was applied to the image and the 
	//biggest connected component which contained the goalpost bars was obtained.
	resultImage = image1.findBiggestConnectedComponent(resultImage);
	//Gaussian filter was applied to the image so as to remove the ropes connections from the
	//goalpost. Doing canny edge detection after the gaussian blur removes the weak links between 
	//the ropes of the goalpost and brings out the strong edges of the goalpost
	resultImage = image1.applyGaussianAndCannyEdge(resultImage);
	// Initially Harris corner detector was used but it detected too many false positives in the image
	//Therefore, it was removed and hough transform was used.Equations of lines around the goal post were 
	//made out of the results of hough and the intersection points were retrived out these equations. These intersection
	//points were further processed to find the final points which are plotted in the image
	resultImage = image1.applyHoughTransformAndFindPoints(resultImage);
	
	image1.displayImage(resultImage);
	//Some of the images needed cleaning before getting processed.For example a grainy old image This image 
	//could used poisson hole filling algorithm in order to remove bloches on the image. Applying histogram
	//equalization and gaussian blur after he poisson hole filling might restore the image to a good condition
	// in order for it to be processed for goalpost detection. 
	
	//imwrite("imageWithGoal0020.png",resultImage);
	
	
	return 0;
}
