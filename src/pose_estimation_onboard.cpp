#include <iostream>

// ros
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"

// for maintaining drone state and aggregating navdata
class Drone {
    public:
        int altd;
        double x, y, z;
        double roll, pitch, yaw;
        Drone();
        void nav_cb(const ardrone_autonomy::NavdataConstPtr& msg);
};

Drone::Drone() {
    altd = 0;
    x = y = z = 0;
    roll = pitch = yaw = 0;
};

void Drone::nav_cb(const ardrone_autonomy::NavdataConstPtr& msg) {
    x += msg->vx;
    y += msg->vy;
    z += msg->vz;
}

int main(int argc, char **argv) {
    // for drone state
    Drone *drone = new Drone();

    // ros initialization
    ros::init(argc, argv, "pose_estimation_onboard");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // retrieve sensor data
    ros::Subscriber navdata;
    navdata = n.subscribe("ardrone/navdata", 5, &Drone::nav_cb, drone);
    
    while(ros::ok()) {
        std::cout << "x: " << drone->x << std::endl;
        std::cout << "y: " << drone->y << std::endl;
        std::cout << "z: " << drone->z << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    // clean up
    delete drone;
}
