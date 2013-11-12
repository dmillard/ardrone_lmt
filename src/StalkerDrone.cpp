/*
 * StalkerDrone.cpp
 *
 *  Created on: Nov 4, 2013
 *      Author: nlichtenberg
 */

// TODO: Filter lines by angle to neighbors

#include "StalkerDrone.h"

using namespace cv;
int hough_edge_thresh = 80;
int hough_max_edge_thresh = 255;
int hough_space_resolution = 5;
int hough_space_resolution_max = 500;
int hough_angle_resolution = 45;
int hough_angle_resolution_max = 180;
int hough_gap = 40;
int hough_gap_max = 500;
int hough_min_len = 30;
int hough_min_len_max = 500;
int num_erosions = 6;
int num_dilations = 2;
int angle_tolerance = 20;
int c;
float max_vel = 0.3;
int reference_face = 0;
int mean_face = 0;
bool show_frame = true;
IplImage main_window;


StalkerDrone::StalkerDrone() {
	// TODO Auto-generated constructor stub
	init();
}

StalkerDrone::~StalkerDrone() {
	// TODO Auto-generated destructor stub
	cvDestroyWindow("Main");
}

struct SPoint{
	int x;
	int y;
	int weight;
};

bool compareSPointWeights (SPoint i,SPoint j) { return (i.weight > j.weight); }
bool orderX (Point i,Point j) { return (i.x < j.x); }
bool order_low_to_high_Y (Point i,Point j) { return (i.y < j.y); }
bool order_high_to_low_Y (Point i,Point j) { return (i.y > j.y); }

cv::Point mean_point(std::vector<cv::Point> points){
	Point mean;
	for (int i = 0; i < points.size(); i++){
		mean.x += points.at(i).x;
		mean.y += points.at(i).y;
	}
	mean.x /= points.size();
	mean.y /= points.size();
	return mean;
}

float distance (SPoint p1, SPoint p2){
	double val = (double)((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
	return cv::sqrt(val);
}

float distance (int x1, int y1, int x2, int y2){
	double val = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	return cv::sqrt(val);
}

bool lines_touch (cv::Vec4i l1, cv::Vec4i l2){


	float radius = 50;
	bool result = distance(l1[0], l1[1], l2[0], l2[1]) < radius
			|| distance(l1[0], l1[1], l2[2], l2[3]) < radius
			|| distance(l1[2], l1[3], l2[0], l2[1]) < radius
			|| distance(l1[2], l1[3], l2[2], l2[3]) < radius;


	return result;
}

float angle (cv::Point v1, cv::Point v2){
	float angle;
	float vec1_len = cv::sqrt(v1.x*v1.x + v1.y*v1.y);
	float vec2_len = cv::sqrt(v2.x*v2.x + v2.y*v2.y);
	double val= (v1.x * v2.x + v1.y * v2.y) / (vec1_len * vec2_len);
	angle = std::acos(val) / 3.142 * 180;
	return angle;
}

float angle (cv::Vec4i v1, cv::Vec4i v2){
	float angle;

	cv::Vec2i vec1(v1[0] - v1[2], v1[1] - v1[3]);
	cv::Vec2i vec2(v2[0] - v2[2], v2[1] - v2[3]);
	float vec1_len = cv::sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1]);
	float vec2_len = cv::sqrt(vec2[0]*vec2[0] + vec2[1]*vec2[1]);
	double val= (vec1[0] * vec2[0] + vec1[1] * vec2[1]) / (vec1_len * vec2_len);
	angle = std::acos(val) / 3.142 * 180;
	return angle;
}

std::vector<cv::Vec4i> filter_lines(std::vector<cv::Vec4i> noised_lines){

	int radius = 20;
	std::vector<cv::Vec4i> filtered_lines;
	std::vector<cv::Vec4i> working_set;

	working_set = noised_lines;


	for (int i = 0; i < noised_lines.size(); i++){
		cv::Vec4i reference = working_set.back();
		working_set.pop_back();

		// search for other lines that are connected within radius and are perpendicular to reference
		for(int k = 0; k < working_set.size(); k++){
			if(lines_touch(reference, working_set.at(k))){
				// if line touch, see if they are perpendicular


				if(angle(reference, working_set.at(k)) > 70 || angle(reference, working_set.at(k)) < 110){
					filtered_lines.push_back(reference);
					break;
				}
			}
		}
	}
	// try to find sets of four lines in rectangular shape
	// find the next line

	return filtered_lines;
}



geometry_msgs::Twist StalkerDrone::processImg(cv::Mat src){
	geometry_msgs::Twist result;
	result.linear.x = 0;
	result.linear.y = 0;
	result.linear.z = 0;
	result.angular.x = 0;
	result.angular.y = 0;
	result.angular.z = 0;
	cv::Mat src_gray;
	cv::Mat dst, cdst, ldst, detected_edges;

	std::vector<cv::Vec4i> lines;
	std::vector<cv::Vec4i> filtered_lines;
	std::vector<SPoint> points;
	std::vector<std::vector<SPoint> > point_group;
	int const lowThreshold = 60;
	int const maxThreshold = 160;
	int const pointgroup_radius = 15;
	int erosion_size = 1;
	int kernel_size = 3;

	cv::Mat line_points = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
	cv::cvtColor( src, src_gray, CV_BGR2GRAY );
	/// Reduce noise with a kernel 3x3
	cv::GaussianBlur( src_gray, detected_edges, cv::Size(3,3), 1, 1 );
	/// Canny detector
	cv::Canny( detected_edges, detected_edges, lowThreshold, maxThreshold, kernel_size );
	/// Using Canny's output as a mask, we display our result

	// process the edges, getrid of single edges
	cv::Mat element;
	element = cv::getStructuringElement( MORPH_CROSS,
										   cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
										   cv::Point( erosion_size, erosion_size ) );
	// Apply the erosion operation
	cv::dilate( detected_edges, detected_edges, element, Point(-1,-1), num_dilations );
	cv::erode( detected_edges, detected_edges, element, Point(-1,-1), num_erosions );

	src.copyTo( dst, detected_edges);
	cv::cvtColor( dst, cdst, CV_BGR2GRAY );
	cv::HoughLinesP( cdst, lines, hough_space_resolution, CV_PI/hough_angle_resolution, hough_edge_thresh, hough_min_len, hough_gap);

	lines = filter_lines(lines);

	for( size_t i = 0; i < lines.size(); i++ )
	{
		SPoint p1;
		SPoint p2;
		int x1 = lines[i][0];
		int x2 = lines[i][2];
		int y1 = lines[i][1];
		int y2 = lines[i][3];
		p1.x = x1;
		p1.y = y1;
		p2.x = x2;
		p2.y = y2;
		points.push_back(p1);
		points.push_back(p2);
	}

	// form groups of linepoints
	// pick a point and transfer it to on of the point groups.
	// add points within distance to same group
	// repeat until lines is empty
	while(!points.empty()){
		std::vector<SPoint> group;
		SPoint reference(points.back());
		points.pop_back();
		group.clear();
		group.push_back(reference);

		for(int i = 0; i < points.size(); i++){

			if(distance(reference, points.at(i)) <= pointgroup_radius){
				group.push_back(points.at(i));
				points.erase(points.begin() + i);
			}
		}
		point_group.push_back(group);
		cv::circle( detected_edges, cv::Point(group.front().x, group.front().y), pointgroup_radius,  cv::Scalar(255, 0, 0), 2, 8, 0 );
	}
	// get mean of pointgroups to create single points
	// add weight
	std::vector<SPoint> meanPoints;
	for(int i = 0; i < point_group.size(); i ++){
		std::vector<SPoint> group = point_group.at(i);
		meanPoints.push_back(SPoint());
		meanPoints.back().x = 0;
		meanPoints.back().y = 0;
		meanPoints.back().weight = 0;
		for(int k = 0; k < group.size(); k++){
			SPoint point = group.at(k);
			meanPoints.back().x += point.x;
			meanPoints.back().y += point.y;
			meanPoints.back().weight += 1;
		}
		meanPoints.back().x /= meanPoints.back().weight;
		meanPoints.back().y /= meanPoints.back().weight;
	}

	// sort the points
	std::sort (meanPoints.begin(), meanPoints.end(), compareSPointWeights);

	// sort out points that are too close
	if(meanPoints.size() > 1)
	for(int i = 0; i < meanPoints.size() - 1; i++){
		for(int k = i + 1; k < meanPoints.size(); k++){
			if(distance(meanPoints.at(i), meanPoints.at(k)) <= pointgroup_radius*2)
				meanPoints.erase(meanPoints.begin() + k);
		}
	}

	if(meanPoints.size() > 7){
		// found 8 points or more
		// get the best 8 to create rectangles
		std::vector<Point> winners;
		winners.push_back(Point(meanPoints.at(0).x, meanPoints.at(0).y));
		winners.push_back(Point(meanPoints.at(1).x, meanPoints.at(1).y));
		winners.push_back(Point(meanPoints.at(2).x, meanPoints.at(2).y));
		winners.push_back(Point(meanPoints.at(3).x, meanPoints.at(3).y));
		winners.push_back(Point(meanPoints.at(4).x, meanPoints.at(4).y));
		winners.push_back(Point(meanPoints.at(5).x, meanPoints.at(5).y));
		winners.push_back(Point(meanPoints.at(6).x, meanPoints.at(6).y));
		winners.push_back(Point(meanPoints.at(7).x, meanPoints.at(7).y));

		// order points from left to right
		// first 4 are first rectangle
		std::sort (winners.begin(), winners.end(), orderX);
		std::sort (winners.begin(), winners.begin()+2, order_low_to_high_Y);
		std::sort (winners.begin()+2, winners.begin()+4, order_high_to_low_Y);
		std::sort (winners.begin()+4, winners.begin()+6, order_low_to_high_Y);
		std::sort (winners.begin()+6, winners.end(), order_high_to_low_Y);

		// check if the transform of the two rectangles is affine
		// if not, theracking os not valid
		if(			angle(Point(winners.at(1).x - winners.at(0).x, winners.at(1).y - winners.at(0).y),
					Point(winners.at(5).x - winners.at(4).x, winners.at(5).y - winners.at(4).y)) > angle_tolerance
				||	angle(Point(winners.at(2).x - winners.at(1).x, winners.at(2).y - winners.at(1).y),
					Point(winners.at(6).x - winners.at(5).x, winners.at(6).y - winners.at(5).y)) > angle_tolerance
				||	angle(Point(winners.at(3).x - winners.at(2).x, winners.at(3).y - winners.at(2).y),
					Point(winners.at(7).x - winners.at(6).x, winners.at(7).y - winners.at(6).y)) > angle_tolerance
				||	angle(Point(winners.at(0).x - winners.at(3).x, winners.at(0).y - winners.at(3).y),
					Point(winners.at(4).x - winners.at(7).x, winners.at(4).y - winners.at(7).y)) > angle_tolerance)
		{
			if(show_frame){
				main_window = IplImage(src);
				cvShowImage("Main", &main_window);
			}
			return result;
		}

		cv::line( src, winners.at(0), winners.at(1), cv::Scalar(0,255,255), 3, 8 );
		cv::line( src, winners.at(1), winners.at(2), cv::Scalar(0,255,255), 3, 8 );
		cv::line( src, winners.at(2), winners.at(3), cv::Scalar(0,255,255), 3, 8 );
		cv::line( src, winners.at(3), winners.at(0), cv::Scalar(0,255,255), 3, 8 );

		cv::line( src, winners.at(4), winners.at(5), cv::Scalar(255,0,255), 3, 8 );
		cv::line( src, winners.at(5), winners.at(6), cv::Scalar(255,0,255), 3, 8 );
		cv::line( src, winners.at(6), winners.at(7), cv::Scalar(255,0,255), 3, 8 );
		cv::line( src, winners.at(7), winners.at(4), cv::Scalar(255,0,255), 3, 8 );

		// get the mean as "focus point" for drone
		Point focus = mean_point(winners);

		cv::line( src, focus, Point(src.cols/2, src.rows/2), cv::Scalar(255,0,0), 3, 8 );
		cv::circle( src, Point(src.cols/2, src.rows/2), 4,  cv::Scalar(255, 0, 0), 6, 8, 0 );
		cv::circle( src, focus, 4,  cv::Scalar(0, 255, 0), 6, 8, 0 );

		// fit actual rectangles into the detected quadrangle
		Point rect1_center = mean_point(vector<Point>(winners.begin(), winners.begin()+4));
		Point rect2_center = mean_point(vector<Point>(winners.begin()+4, winners.end()));

		int rect1_x_size = (winners.at(2).x + winners.at(3).x)/2 - (winners.at(0).x + winners.at(1).x)/2;
		int rect1_y_size = (winners.at(1).y + winners.at(2).y)/2 - (winners.at(0).y + winners.at(3).y)/2;
		int rect2_x_size = (winners.at(6).x + winners.at(7).x)/2 - (winners.at(4).x + winners.at(5).x)/2;
		int rect2_y_size = (winners.at(5).y + winners.at(6).y)/2 - (winners.at(4).y + winners.at(7).y)/2;

		rectangle(src, Point(rect1_center.x - rect1_x_size/2, rect1_center.y - rect1_y_size/2),
				Point(rect1_center.x + rect1_x_size/2, rect1_center.y + rect1_y_size/2),
				cv::Scalar(0,255,0), 2,8,0);

		rectangle(src, Point(rect2_center.x - rect2_x_size/2, rect2_center.y - rect2_y_size/2),
						Point(rect2_center.x + rect2_x_size/2, rect2_center.y + rect2_y_size/2),
						cv::Scalar(0,255,0), 2,8,0);

		int rect1_face = rect1_x_size * rect1_y_size;
		int rect2_face = rect2_x_size * rect2_y_size;
		mean_face = (rect1_x_size + rect2_x_size)/2 * (rect1_y_size + rect2_y_size)/2;

		// take reference for going back or forward
		c = cvWaitKey(10);
		if (c==111)// o
			reference_face = mean_face;
		if (c==112)//p
			show_frame = !show_frame;

		// process the new directions for the drone
		float rect_fraction = (float)rect2_face / (float)rect1_face;

		std::string twist_command = "No calibration found";
		std::string shift_command = "";
		std::string altitude_command = "";
		std::string acc_command = "";
		float z = 0;
		float y = 0;
		float x = 0;
		float turn = 0;

		if(reference_face > 0){
			if(rect_fraction < 1){
				twist_command = "twist right";
			}
			if(rect_fraction > 1){
				twist_command = "twist left";
			}
			if(focus.y > src.rows/2){
				altitude_command = "go down";
			}
			if(focus.y < src.rows/2){
				altitude_command = "go up";
			}
			if(focus.x > src.cols/2){
				shift_command = "go right";
			}
			if(focus.x < src.cols/2){
				shift_command = "go left";
			}
			if(mean_face < reference_face){
				acc_command = "go forward";
			}
			if(mean_face > reference_face){
				acc_command = "go backward";
			}


			z = (float)(src.rows/2 - focus.y) / (src.rows/2) * max_vel;
			y = (float)(src.cols/2 - focus.x) / (src.cols/2) * max_vel;
			x = cv::log((float)reference_face / mean_face) * max_vel;
			turn = cv::log(rect_fraction) * max_vel;

			result.linear.y = y;
			result.linear.z = z;
			result.linear.x = x;
		}

		putText(src, twist_command + " " + boost::lexical_cast<string>(turn), Point(5, 15), FONT_HERSHEY_COMPLEX_SMALL,
						1, cv::Scalar(0,255,0), 1, 8, false );

		putText(src, shift_command + boost::lexical_cast<string>(y), Point(5, 35), FONT_HERSHEY_COMPLEX_SMALL,
								1, cv::Scalar(0,255,0), 1, 8, false );

		putText(src, altitude_command + boost::lexical_cast<string>(z), Point(5, 55), FONT_HERSHEY_COMPLEX_SMALL,
								1, cv::Scalar(0,255,0), 1, 8, false );

		putText(src, acc_command + " " + boost::lexical_cast<string>(x), Point(5, 75), FONT_HERSHEY_COMPLEX_SMALL,
								1, cv::Scalar(0,255,0), 1, 8, false );

		std::stringstream ss;
		ss << rect1_face;
		std::string rect1_val = ss.str();
		putText(src,  rect1_val, rect1_center, FONT_HERSHEY_COMPLEX_SMALL,
				1, cv::Scalar(0,255,0), 1, 8, false );
		ss.str("");
		ss << rect2_face;
		std::string rect2_val = ss.str();
		putText(src,  rect2_val, rect2_center, FONT_HERSHEY_COMPLEX_SMALL,
				1, cv::Scalar(0,255,0), 1, 8, false );
	}

	/// Showing the result
	if(show_frame){
		main_window = IplImage(src);
		//IplImage* second = new IplImage(detected_edges);
		cvShowImage("Main", &main_window); // show frame
		//cvShowImage("Second", second); // show frame
	}
	return result;
}

void StalkerDrone::set_velocity(float vel){
	max_vel = vel;
}

void StalkerDrone::init(){
	cvNamedWindow("Main",0);
	//cv::createTrackbar( "Erosion: ", "Main", &num_erosions, 20);
	//cv::createTrackbar( "Dilation: ", "Main", &num_dilations, 20);
	//cv::createTrackbar( "Angle Tolerance: ", "Main", &angle_tolerance, 90);
}

