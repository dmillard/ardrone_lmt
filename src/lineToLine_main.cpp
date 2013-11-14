/*
 * main.cpp
 *
 *  Created on: Nov 4, 2013
 *      Author: nlichtenberg
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "LineToLine.h"

CvCapture* cv_cap;
int c = 0;
cv_bridge::CvImagePtr cv_ptr;

using namespace cv;

cv::Mat image;
int miss_accu = 0;
int miss_max = 5; // maximum number of unexpected, consecutive tracking results before switching to next step.
std::vector<std::string> step_msgs;
std::vector<bool> expected_tracking;

//Mat rotateImage(const Mat& source, double angle)
//{
//    Point2f src_center(source.cols/2.0F, source.rows/2.0F);
//    Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
//    Mat dst;
//    warpAffine(source, dst, rot_mat, source.size());
//    return dst;
//}


int main(int argc, char **argv)
{
	int step = 0;
	step_msgs.push_back("Seeing first line. Going forward");
	step_msgs.push_back("Left first line. Going forward");
	step_msgs.push_back("Seeing second line. Going forward");
	step_msgs.push_back("Left second line. Going backward");
	step_msgs.push_back("Seeing second line again. Going backward");
	step_msgs.push_back("Left second line again. Going backward");
	step_msgs.push_back("Seeing first line again. Going backwards");
	expected_tracking.push_back(true);
	expected_tracking.push_back(false);
	expected_tracking.push_back(true);
	expected_tracking.push_back(false);
	expected_tracking.push_back(true);
	expected_tracking.push_back(false);
	expected_tracking.push_back(true);
	ros::init(argc, argv, "LineToLine");
   // ros::Rate loop_rate(50);

	cv_cap = cvCaptureFromCAM(0);
	LineToLine* ltl = new LineToLine();

	//image = imread(argv[1], CV_LOAD_IMAGE_COLOR);
	//resize(image, image, Size(image.cols/6, image.rows/6));
	image = cvQueryFrame(cv_cap);

	// wait for the user to start
	printf("Position the drone over the first line and facing to thesecond line, then hit o to go.\n");
	while(true){
		image = cvQueryFrame(cv_cap);
		ltl->processImg(image);
		c = cvWaitKey(10); // wait 10 ms or for key stroke
		if(c == 111)
			break; // if ESC, break and quit
	}

	// drone has to be positioned over first line and must be detecting the line for this to work
	while(true){
		image = cvQueryFrame(cv_cap);
		if(expected_tracking.at(step) != ltl->processImg(image)){
			miss_accu++;
			if(miss_accu == miss_max)
				step++;
		}else
			miss_accu = 0;
		if(step >= step_msgs.size())
				break;
		std::cout << string( 50, '\n' );
		printf("Step %i. \n%s.\n", step, step_msgs.at(step).c_str());
		c = cvWaitKey(10); // wait 10 ms or for key stroke
		if(c == 27)
			break; // if ESC, break and quit
	}
	std::cout << string( 50, '\n' );
	printf("Crossed first line. Stopping and landing.\nPress key to end.\n");
	cvWaitKey();

//	double angle = 5;
//	while (true/*ros::ok()*/) {
//		image = rotateImage(image, angle);
//		//image = cvQueryFrame(cv_cap);
//		if(ltl->processImg(image))
//			printf("Detecting something...\n");
//		else
//			printf("Detecting nothing...\n");
//		c = cvWaitKey(10); // wait 10 ms or for key stroke
//		if(c == 27)
//			break; // if ESC, break and quit
//		//loop_rate.sleep();
//	}

	/* clean up */
	cvReleaseCapture( &cv_cap );
	delete ltl;
	//ros::spin();
	return 0;
}
