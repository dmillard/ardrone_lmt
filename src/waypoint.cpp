#include <iostream>

// ros
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"

#define VEL .5

typedef struct { double x, y, z; } waypoint_t;

class DroneController {
    public:
        waypoint_t wp;
        geometry_msgs::Twist twist;
        ros::Publisher launch, land, reset, cmd_vel;
        DroneController(ros::NodeHandle n);
        void pose_cb(tum_ardrone::filter_stateConstPtr& msg);
};

DroneController::DroneController(ros::NodeHandle n) {
    // set up topics
    launch = n.advertise<std_msgs::Empty>("ardrone/takeoff", 5);
    land = n.advertise<std_msgs::Empty>("ardrone/land", 5);
    reset = n.advertise<std_msgs::Empty>("ardrone/reset", 5);
    cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 15);
    
    // initialize twist
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
}

void DroneController::pose_cb(tum_ardrone::filter_stateConstPtr& msg) {
    // pose/waypoint deltas
    double dx = wp.x - msg.x;
    double dy = wp.y - msg.y;
    double dz = wp.z - msg.z;

    // normalization constant
    double norm = VEL/(dx*dx+dy*dy+dz*dz);

    // set and send
    twist.linear.x = dx*norm;
    twist.linear.y = dy*norm;
    twist.linear.z = dz*norm;
    cmd_vel.publish(twist);
}

int main(int argc, char **argv) {
    // ros init
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    std_msgs::Empty empty;

    // setup drone control
    DroneControl dc(n);

    // for feedback
    ros::Subscriber pose;
    pose n.subscribe("ardrone/predictedPose", 5, &Drone::pose_cb, dc);

    // go
    ros::spin();
}
