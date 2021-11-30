#include <iostream>
#include <serial.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <lidar.h>
#include <uart.h>
#include "ros/ros.h"
#include <mutex>
#include <cmath>
#include <geometry_msgs/Twist.h>


int uart_thread_entry(){
	ros::NodeHandle n;
    ros::Rate loop_rate(100);
	while(ros::ok()){
        //send sol to uart
        send_pose_to_serial(pub_to_controller[0],pub_to_controller[1],pub_to_controller[2],0.0,0.0,0.0,0.0,0.0,0.0,0.0);
	}
}
