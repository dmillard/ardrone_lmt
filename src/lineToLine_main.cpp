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
#include "LineToLine.h".h"

CvCapture* cv_cap;
int c = 0;
cv_bridge::CvImagePtr cv_ptr;

using namespace cv;

cv::Mat image;



int main(int argc, char **argv)
{
	ros::init(argc, argv, "LineToLine");
	//ros::NodeHandle node;
	//image_transport::ImageTransport it(node);
	//image_transport::Subscriber img_subs = it.subscribe("ardrone/front", 1, imageCallback);

    //ros::Rate loop_rate(50);
	cv_cap = cvCaptureFromCAM(0);
	LineToLine* ltl = new LineToLine();



	while (true/*ros::ok()*/) {

		image = cvQueryFrame(cv_cap);
		if(ltl->processImg(image))
			printf("Detecting something...\n");
		else
			printf("Detecting nothing...\n");
		c = cvWaitKey(10); // wait 10 ms or for key stroke
		if(c == 27)
			break; // if ESC, break and quit
		//loop_rate.sleep();
	}

	/* clean up */
	cvReleaseCapture( &cv_cap );
	delete ltl;
	//ros::spin();
	return 0;
}
