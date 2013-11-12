#include <iostream>
#include <ncurses.h>

// ros
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

#define VEL .5;

int main(int argc, char **argv) {
    // ros initialization
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // connect to ros to control drone
    ros::Publisher launch, land, reset, cmd_vel;
    launch = n.advertise<std_msgs::Empty>("ardrone/takeoff", 5);
    land = n.advertise<std_msgs::Empty>("ardrone/land", 5);
    reset = n.advertise<std_msgs::Empty>("ardrone/reset", 5);
    cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 15);

    // set up control messages
    std_msgs::Empty empty;
    geometry_msgs::Twist twist;
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;

    // ncurses initialization
    initscr();
    raw();
    noecho();
    keypad(stdscr, TRUE);
    timeout(500);

    printw("Team LMT ArDrone Keyboard Control Node\n\n");
    printw("Usage:\n");
    printw("    r - Reset\n");
    printw("    q - Quit (and land if necessary)\n");
    printw("    Space - Takeoff/Land\n");
    printw("    Arrow keys - Move laterally\n");
    printw("    w/s - Move up/down\n");
    printw("    a/d - Rotate left/right\n");
    refresh();

    // process keyboard input
    int in_air = 0, running = 1;
    while(running) {
        int ch = getch();
        switch(ch) {
            case ERR:
                cmd_vel.publish(twist);
                break;
            case 'q':
            case 'Q':
                cmd_vel.publish(twist);
                land.publish(empty);
                running = 0;
                break;
            case ' ':
                cmd_vel.publish(twist);
                if(in_air) land.publish(empty);
                else launch.publish(empty);
                in_air = !in_air;
                break;
            case 'r':
            case 'R':
                reset.publish(empty);
                break;
            case KEY_UP:
                twist.linear.x = VEL;
                cmd_vel.publish(twist);
                twist.linear.x = 0;
                break;
            case KEY_DOWN:
                twist.linear.x = -VEL;
                cmd_vel.publish(twist);
                twist.linear.x = 0;
                break;
            case KEY_LEFT:
                twist.linear.y = VEL;
                cmd_vel.publish(twist);
                twist.linear.y = 0;
                break;
            case KEY_RIGHT:
                twist.linear.y = -VEL;
                cmd_vel.publish(twist);
                twist.linear.y = 0;
                break;
            case 'w':
            case 'W':
                twist.linear.z = VEL;
                cmd_vel.publish(twist);
                twist.linear.z = 0;
                break;
            case 's':
            case 'S':
                twist.linear.z = -VEL;
                cmd_vel.publish(twist);
                twist.linear.z = 0;
                break;
            case 'a':
            case 'A':
                twist.angular.z = -VEL;
                cmd_vel.publish(twist);
                twist.angular.z = 0;
                break;
            case 'd':
            case 'D':
                twist.angular.z = VEL;
                cmd_vel.publish(twist);
                twist.angular.z = 0;
                break;
        }
    }

    // end curses
    endwin();
}
