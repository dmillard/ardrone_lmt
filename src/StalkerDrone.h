/*
 * StalkerDrone.h
 *
 *  Created on: Nov 4, 2013
 *      Author: nlichtenberg
 */

#ifndef STALKERDRONE_H_
#define STALKERDRONE_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdlib.h>
#include "geometry_msgs/Twist.h"

class StalkerDrone {
public:
	StalkerDrone();
	virtual ~StalkerDrone();

	geometry_msgs::Twist processImg(cv::Mat src);
	void set_velocity(float vel);

private:
	void init();
};

#endif /* STALKERDRONE_H_ */
