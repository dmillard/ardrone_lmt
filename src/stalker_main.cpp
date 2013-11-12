// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

// cv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// us
#include "StalkerDrone.h"

class Controller {
    public:
        Controller(ros::NodeHandle n, StalkerDrone *s);
        void cam_cb(const sensor_msgs::ImageConstPtr& img);
    private:
        size_t counter;
        ros::Publisher cmd;
        geometry_msgs::Twist twist;
        StalkerDrone *sd;
};

Controller::Controller(ros::NodeHandle n, StalkerDrone *s) {
    cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    counter = 0;
    sd = s;
}

void Controller::cam_cb(const sensor_msgs::ImageConstPtr& img) {
    if(counter++ == 10) {
        counter = 0;
        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img);
        cv::resize(cv_img->image, cv_img->image, cv::Size(640, 360), 0, 0);
        twist = sd->processImg(cv_img->image);
        cmd.publish(twist);
    }
}

int main(int argc, char **argv) {
    cv::Mat image;
    ros::init(argc, argv, "stalker");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    StalkerDrone *sd = new StalkerDrone();    
    Controller *dc = new Controller(n, sd);
    sd->set_velocity(0.3);

    ros::Subscriber cam;
    cam = n.subscribe("ardrone/front/image_raw", 5, &Controller::cam_cb, dc);
    
    ros::spin();

    return 0;
}
