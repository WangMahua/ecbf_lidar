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

using namespace std;

imu_t imu;
float rc_value[4]; // roll pitch yaw throttle 
int rc_ecbf_mode;

uint8_t generate_imu_checksum_byte(uint8_t *payload, int payload_count){
	uint8_t result = IMU_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

int imu_decode(uint8_t *buf){
	static float x_array_uart[100];
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_imu_checksum_byte(&buf[3], IMU_SERIAL_MSG_SIZE - 3);
	if(checksum != recv_checksum) {
		return 1; //error detected
	}
	float roll, pitch, yaw, throttle;
    int rc_ch7;

	memcpy(&roll, &buf[2], sizeof(float)); //in ned coordinate system
	memcpy(&pitch, &buf[6], sizeof(float));
	memcpy(&yaw, &buf[10], sizeof(float));

	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&throttle, &buf[14], sizeof(float));
	memcpy(&rc_ch7, &buf[18], sizeof(int));
	//memcpy(&imu.gyrop[0], &buf[22], sizeof(float));

	rc_value[0] = roll; //east
	rc_value[1]= pitch; //north
	rc_value[2] = 0; //up //do not change yaw angle
	rc_value[3] = throttle;
	rc_ecbf_mode = rc_ch7;

	return 0;
}

void imu_buf_push(uint8_t c){
	if(imu.buf_pos >= IMU_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < IMU_SERIAL_MSG_SIZE; i++) {
			imu.buf[i - 1] = imu.buf[i];
		}

		/* save new byte to the last array element */
		imu.buf[IMU_SERIAL_MSG_SIZE - 1] = c;
		imu.buf_pos = IMU_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		imu.buf[imu.buf_pos] = c;
		imu.buf_pos++;
	}
}

/* decode rc value from ncrl-flight-control board*/
int uart_thread_entry(){
	ros::NodeHandle n;
    ros::Rate loop_rate(100);
	char c;
	imu.buf_pos = 0;

	cout<<"start\n";
	while(ros::ok()){
		if(serial_getc(&c) != -1) {
			imu_buf_push(c); 
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+'){
				if(imu_decode(imu.buf)==0){
					
				}
			}
		}
	}
	return 0;

}
