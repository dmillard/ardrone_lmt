/*
 * LineToLine.cpp
 *
 *  Created on: Nov 4, 2013
 *      Author: nlichtenberg
 */


#include "LineToLine.h"

bool show_frame = true;

int dominance = 40;
int thresh = 10; //proportion of foreground

LineToLine::LineToLine() {
	init();
}

LineToLine::~LineToLine() {
	cvDestroyWindow("Main");
}

using namespace cv;
bool LineToLine::processImg(cv::Mat src){
	int accu = 0;
	IplImage resultImage;
	IplImage sourceImage;
	IplImage skelImage;
	float dominance_factor = (float) dominance/100 + 1;
	cv::Mat src_gray (src.size(), CV_8U);
	cv::Mat src_bw (src.size(), CV_8U);
	for(int x = 0; x < src.cols; x++)
		for(int y = 0; y < src.rows; y++){
			if(src.data[y*src.cols*3 + x*3] > src.data[y*src.cols*3 + x*3 + 1] * dominance_factor
			    && src.data[y*src.cols*3 + x*3] > src.data[y*src.cols*3 + x*3 + 2] * dominance_factor){
				src_gray.data[y*src.cols + x] = 255;
				accu++;
			}
			else
				src_gray.data[y*src.cols + x] = 0;
		}
	Mat skel(src.size(), CV_8UC1, cv::Scalar(0));
	Mat temp(src.size(), CV_8UC1);
	Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));


		// Apply the erosion operation
	cv::dilate( src_gray, src_gray, element, Point(-1,-1), 10 );
	cv::erode( src_gray, src_gray, element, Point(-1,-1), 60 );

	// skeletization
//	Mat gray_temp = src_gray.clone();
//	bool done = false;
//	while(!done)
//	{
//	  cv::morphologyEx(gray_temp, temp, cv::MORPH_OPEN, element);
//	  cv::bitwise_not(temp, temp);
//	  cv::bitwise_and(gray_temp, temp, temp);
//	  cv::bitwise_or(skel, temp, skel);
//	  cv::erode(gray_temp, gray_temp, element);
//
//	  double max;
//	  cv::minMaxLoc(gray_temp, 0, &max);
//	  done = (max == 0);
//	}

	/// Showing the result
	if(show_frame){
		resultImage = IplImage(src_gray);
		sourceImage = IplImage(src);
		//skelImage = IplImage(skel);
		cvShowImage("Result", &resultImage); // show frame
		cvShowImage("Source", &sourceImage); // show frame
		//cvShowImage("Skel", &skelImage); // show frame
	}

	if((float)accu/(src.rows*src.cols) > (float)thresh/100)
		return true;
	return false;
}


void LineToLine::init(){
	cvNamedWindow("Source",0);
	cvNamedWindow("Result",0);
	cvNamedWindow("Skel",0);
	cv::createTrackbar( "Blue Dominance: ", "Result", &dominance, 200);
	cv::createTrackbar( "Threshold: ", "Result", &thresh, 100);
	//cv::createTrackbar( "Angle Tolerance: ", "Main", &angle_tolerance, 90);
}

