/*
 * LineToLine.cpp
 *
 *  Created on: Nov 4, 2013
 *      Author: nlichtenberg
 */


#include "LineToLine.h"

bool show_frame = true;
int hough_edge_thresh = 100;
int hough_max_edge_thresh = 255;
int hough_space_resolution = 10;
int hough_space_resolution_max = 500;
int hough_angle_resolution = 90;
int hough_angle_resolution_max = 180;
int hough_gap = 0;
int hough_gap_max = 500;
int hough_min_len = 30;
int hough_min_len_max = 500;
int min_len_factor = 7;
std::vector<cv::Vec4i> lines;

int dominance = 25;
int thresh = 10; //proportion of foreground

LineToLine::LineToLine() {
	init();
}

LineToLine::~LineToLine() {
	cvDestroyWindow("Main");
}

using namespace cv;
bool LineToLine::processImg(cv::Mat src){
	Mat src_copy = src.clone();
	int accu = 0;
	IplImage resultImage;
	IplImage sourceImage;
	IplImage skelImage;
	float dominance_factor = (float) dominance/100 + 1;
	cv::Mat src_gray (src_copy.size(), CV_8U);
	cv::Mat src_bw (src_copy.size(), CV_8U);
	for(int x = 0; x < src_copy.cols; x++)
		for(int y = 0; y < src_copy.rows; y++){
			if(src.data[y*src_copy.cols*3 + x*3] > src_copy.data[y*src.cols*3 + x*3 + 1] * dominance_factor
			    && src.data[y*src_copy.cols*3 + x*3] > src_copy.data[y*src.cols*3 + x*3 + 2] * dominance_factor){
				src_gray.data[y*src_copy.cols + x] = 255;
				accu++;
			}
			else
				src_gray.data[y*src.cols + x] = 0;
		}
	//Mat skel(src.size(), CV_8UC1, cv::Scalar(0));
	//Mat temp(src.size(), CV_8UC1);
	//Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	hough_min_len = src.rows * min_len_factor / 10;
	cv::HoughLinesP( src_gray, lines, hough_space_resolution, CV_PI/hough_angle_resolution, hough_edge_thresh, hough_min_len, hough_gap);
	if(lines.size() > 0){
		Vec4i l = lines[0];
		line( src_copy, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 3, CV_AA);
	}

	/// Showing the result
	if(show_frame){
		resultImage = IplImage(src_gray);
		sourceImage = IplImage(src_copy);
		//skelImage = IplImage(skel);
		cvShowImage("Result", &resultImage); // show frame
		cvShowImage("Source", &sourceImage); // show frame
		//cvShowImage("Skel", &skelImage); // show frame
	}

	//if((float)accu/(src.rows*src.cols) > (float)thresh/100)
	if(lines.size() > 0)
		return true;
	return false;
}


void LineToLine::init(){
	cvNamedWindow("Source",0);
	cvNamedWindow("Result",0);
	//cvNamedWindow("Skel",0);
	cv::createTrackbar( "Blue Dominance: ", "Result", &dominance, 200);
	cv::createTrackbar( "Threshold: ", "Result", &thresh, 100);
	cv::createTrackbar( "Angle Resolution: ", "Result", &hough_angle_resolution, hough_angle_resolution_max);
	cv::createTrackbar( "Length Factor: ", "Result", &min_len_factor, 10);
}

