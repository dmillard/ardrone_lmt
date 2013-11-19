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
#include "std_msgs/Empty.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "LineToLine.h"

class Controller {
    public:
        bool data_present;
        cv::Mat image;
        void cam_cb(const sensor_msgs::ImageConstPtr& msg) {
            data_present = true;
            this->image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
};

CvCapture* cv_cap;
int c = 0;
cv_bridge::CvImagePtr cv_ptr;

using namespace cv;

int miss_accu = 0;
int miss_max = 5; // maximum number of unexpected, consecutive tracking results before switching to next step.
std::vector<std::string> step_msgs;
std::vector<double> xvel;
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
    ros::init(argc, argv, "linetoline");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    Controller con;
    con.data_present = false;
    cv::Mat image;

    ros::Publisher cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 5);
    ros::Publisher landing = n.advertise<std_msgs::Empty>("/ardrone/land", 5);
    ros::Subscriber cam = n.subscribe("/ardrone/bottom/image_raw", 5, &Controller::cam_cb, &con);

	int step = 0;
	step_msgs.push_back("Seeing first line. Going forward");
    xvel.push_back(.1);
	step_msgs.push_back("Left first line. Going forward");
    xvel.push_back(.1);
	step_msgs.push_back("Seeing second line. Going forward");
    xvel.push_back(.1);
	step_msgs.push_back("Left second line. Going backward");
    xvel.push_back(-.1);
	step_msgs.push_back("Seeing second line again. Going backward");
    xvel.push_back(-.1);
	step_msgs.push_back("Left second line again. Going backward");
    xvel.push_back(-.1);
	step_msgs.push_back("Seeing first line again. Going backwards");
    xvel.push_back(-.1);
	expected_tracking.push_back(true);
	expected_tracking.push_back(false);
	expected_tracking.push_back(true);
	expected_tracking.push_back(false);
	expected_tracking.push_back(true);
	expected_tracking.push_back(false);
	expected_tracking.push_back(true);

	cv_cap = cvCaptureFromCAM(0);
	LineToLine* ltl = new LineToLine();

	//image = imread(argv[1], CV_LOAD_IMAGE_COLOR);
	//resize(image, image, Size(image.cols/6, image.rows/6));
	image = cvQueryFrame(cv_cap);

	// wait for the user to start
	printf("Position the drone over the first line and facing to thesecond line, then hit o to go.\n");
    
    while(!con.data_present) ros::spinOnce();
    
	while(true){
        ros::spinOnce();
        image = con.image;
		ltl->processImg(image);
		c = cvWaitKey(10); // wait 10 ms or for key stroke
		if(c == 111)
			break; // if ESC, break and quit
	}

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

	// drone has to be positioned over first line and must be detecting the line for this to work
	while(true){
        ros::spinOnce();
        image = con.image;
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
        twist.linear.x = xvel.at(step);
        cmd.publish(twist);
		c = cvWaitKey(10); // wait 10 ms or for key stroke
		if(c == 27)
			break; // if ESC, break and quit
	}
	std::cout << string( 50, '\n' );
	printf("Crossed first line. Stopping and landing.\nPress key to end.\n");
    twist.linear.x = 0;
    std_msgs::Empty empty;
    landing.publish(empty);
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
