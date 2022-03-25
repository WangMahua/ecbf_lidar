#include<iostream>
#include<thread>
#include"ros/ros.h"
#include<serial.hpp>
#include"thread/main_thread.h"

using namespace std;

main(int argc ,char **argv){

	ros::init(argc,argv,"ecbf_uart");
	
	cout << "main start" << endl;           
	serial_init((char *)"/dev/ttyUSB0", 115200);
	std::thread lidar_thread(lidar_thread_entry);	//get_imu data from stm32
	std::thread uart_thread(uart_thread_entry);
	lidar_thread.join();
	uart_thread.join();

	return 0;
}
