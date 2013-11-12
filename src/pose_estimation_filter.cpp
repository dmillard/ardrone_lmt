#include <iostream>
#include <algorithm>

// ros
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include "ardrone_autonomy/Navdata.h"

// for maintaining drone state and aggregating navdata
class Drone {
    public:
        int altd;
        double x, y, z;
        double vx, vy, vz;
        double roll, pitch, yaw;

        int pass, lpfi;
        double lpfx, lpfy, lpfz;
        double *lpfa[3];

        Drone(); ~Drone();
        void imu_cb(const sensor_msgs::ImuConstPtr& msg);
    private:
        bool data_collected;
};

Drone::Drone() {
    altd = 0;
    x = y = z = 0;
    vx = vy = vz = 0;
    roll = pitch = yaw = 0;
    lpfi = 0;
    lpfx = lpfy = lpfz = 0;
    data_collected = false;
    ros::param::param<int>("~filter_pass", pass, 100);
    for(int i = 0; i < 3; i++) {
        lpfa[i] = new double[pass];
        for(int j = 0; j < pass; j++) lpfa[i][j] = 0;
    }
};

Drone::~Drone() {
    for(int i = 0; i < 3; i++) delete[] lpfa[i];
}

void Drone::imu_cb(const sensor_msgs::ImuConstPtr& msg) {
    // running average, adaptive bias estimation
    lpfx += (msg->linear_acceleration.x - lpfa[0][lpfi])/pass;
    lpfa[0][lpfi] = msg->linear_acceleration.x;
    lpfy += (msg->linear_acceleration.y - lpfa[1][lpfi])/pass;
    lpfa[1][lpfi] = msg->linear_acceleration.y;
    lpfz += (msg->linear_acceleration.z - lpfa[2][lpfi])/pass;
    lpfa[2][lpfi] = msg->linear_acceleration.z;
    lpfi = (lpfi + 1) % pass;
    if(!data_collected && lpfi == pass - 1) data_collected = true;

    std::cout << lpfx << " " << lpfy << " " << lpfz << std::endl;

    // integrate bias-corrected imu data
    if(data_collected) {
        vx += msg->linear_acceleration.x - lpfx;
        vy += msg->linear_acceleration.y - lpfy;
        vz += msg->linear_acceleration.z - lpfz;
    }

    std::cout << "    ";
    std::cout << msg->linear_acceleration.x - lpfx << " ";
    std::cout << msg->linear_acceleration.y - lpfy << " ";
    std::cout << msg->linear_acceleration.z - lpfz << std::endl;

    // integrate velocity
    x += vx;
    y += vy;
    z += vz;
}

int main(int argc, char **argv) {
    // for drone state
    Drone *drone = new Drone();

    // ros initialization
    ros::init(argc, argv, "pose_estimation_filter");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // retrieve sensor data
    ros::Subscriber navdata;
    navdata = n.subscribe("ardrone/imu", 5, &Drone::imu_cb, drone);

    ros::Publisher raw, lpf, vel, pos;
    raw = n.advertise<geometry_msgs::Vector3>("graph/raw", 5);
    lpf = n.advertise<geometry_msgs::Vector3>("graph/lpf", 5);
    vel = n.advertise<geometry_msgs::Vector3>("graph/vel", 5);
    pos = n.advertise<geometry_msgs::Vector3>("graph/pos", 5);
    geometry_msgs::Vector3 msg;
    
    while(ros::ok()) {
        std::cout << "x: " << drone->x << std::endl;
        std::cout << "y: " << drone->y << std::endl;
        std::cout << "z: " << drone->z << std::endl;

        msg.x = drone->lpfx;
        msg.y = drone->lpfy;
        msg.z = drone->lpfz;
        lpf.publish(msg);
        
        msg.x = drone->vx;
        msg.y = drone->vy;
        msg.z = drone->vz;
        vel.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    // clean up
    delete drone;
}
