#include<iostream>
#include<thread>
#include"ros/ros.h"
#include<serial.hpp>
#include"thread/lidar_debug.h"
#include"thread/vel_debug.h"

using namespace std;

main(int argc ,char **argv){

	ros::init(argc,argv,"ecbf_uart_debug");
	
	cout << "main start" << endl;           
	serial_init((char *)"/dev/ttyUSB0", 115200);
	std::thread lidar_thread(lidar_thread_debug_entry);
	std::thread vel_thread(vel_thread_debug_entry);

	lidar_thread.join();
	vel_thread.join();

	return 0;
}
