/*
 * StalkerDrone.h
 *
 *  Created on: Nov 4, 2013
 *      Author: nlichtenberg
 */

#ifndef LINETOLINE_H_
#define LINETOLINE_H_

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

class LineToLine {
public:
	LineToLine();
	virtual ~LineToLine();

	bool processImg(cv::Mat src);

private:
	void init();
};

#endif /* LINETOLINE_H_ */
