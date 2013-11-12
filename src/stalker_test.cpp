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
#include "StalkerDrone.h"

CvCapture* cv_cap;
int key = 0;
cv_bridge::CvImagePtr cv_ptr;

using namespace cv;

cv::Mat image;

//void imageCallback(const sensor_msgs::ImageConstPtr& msg)
//{
//
//    try
//        {
//          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//          image = cv_ptr->image;
//        }
//        catch (cv_bridge::Exception& e)
//        {
//          ROS_ERROR("cv_bridge exception: %s", e.what());
//    }
//}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "StalkerDrone");
    //ros::NodeHandle node;
    //image_transport::ImageTransport it(node);
    //image_transport::Subscriber img_subs = it.subscribe("ardrone/front", 1, imageCallback);

    //ros::Rate loop_rate(50);
    cv_cap = cvCaptureFromCAM(0);
    StalkerDrone* sd = new StalkerDrone();
    sd->set_velocity(0.3);
    geometry_msgs::Twist msg;



    while (true/*ros::ok()*/) {

        image = cvQueryFrame(cv_cap);
        //ros::spinOnce();
        msg = sd->processImg(image);
        key = cvWaitKey(10); // wait 10 ms or for key stroke
        if(key == 27)
            break; // if ESC, break and quit
        //loop_rate.sleep();
    }

    /* clean up */
    cvReleaseCapture( &cv_cap );
    delete sd;
    //ros::spin();
    return 0;
}
