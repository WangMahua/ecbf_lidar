#include<iostream>
#include<thread>
#include"ros/ros.h"
#include<serial.hpp>
#include"thread/vel.h"
#include"thread/lidar.h"
#include"thread/uart.h"

using namespace std;

main(int argc ,char **argv){

	ros::init(argc,argv,"ecbf_uart");
	
	cout << "main start" << endl;           
	serial_init((char *)"/dev/ECBF_UART", 115200);
	std::thread lidar_thread(lidar_thread_entry);	//get_imu data from stm32
	std::thread vel_thread(vel_thread_entry);	//push imu data to ROS and recieve position data from ROS
	std::thread uart_thread(uart_thread_entry);
	lidar_thread.join();
	vel_thread.join();
	uart_thread.join();

	return 0;
}
