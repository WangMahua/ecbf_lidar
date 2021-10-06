#include <iostream>
#include <serial.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <lidar.h>
#include "ros/ros.h"
#include <mutex>
#include <cmath>
#include "osqp.h"
#include <geometry_msgs/Twist.h>
#include "vel.h"


#define upper_rc 35
#define lower_rc -35

#define PATH_MODE 1 /* 0:box 1:tube */

#define K1 5
#define K2 3
#define ECBF_THESHOLD 0.1

//box
#define X_UPPER_BOUND 1
#define X_LOWER_BOUND -1
#define Y_UPPER_BOUND 1
#define Y_LOWER_BOUND -1
#define Z_UPPER_BOUND 100
#define Z_LOWER_BOUND 0.5

//tube
#define R_MAX 1.3
#define R_MIN 0.5
#define Z_UPPER_BOUND 100
#define Z_LOWER_BOUND 0.5

using namespace std;

mutex imu_mutex;
imu_t imu;
int rc_ch7;
rc_data rc;



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

	memcpy(&rc.roll, &buf[2], sizeof(float)); //in ned coordinate system
	memcpy(&rc.pitch, &buf[6], sizeof(float));
	memcpy(&rc.yaw, &buf[10], sizeof(float));

	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&rc.throttle, &buf[14], sizeof(float));
	memcpy(&rc.mode, &buf[18], sizeof(int));
	
	return 0;
}

void imu_buf_push(uint8_t c)
{
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


#define g 9.81

int imu_thread_entry(){
 
	ros::NodeHandle n;
	ros::Publisher qp_pub = n.advertise<geometry_msgs::Twist>("qp", 1); 
	ros::Publisher debug_rc_pub = n.advertise<geometry_msgs::Twist>("rc_info", 1); 
	ros::Publisher debug_qp_pub = n.advertise<geometry_msgs::Twist>("qp_info", 1); 
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3>("vel_info", 1); 
	
	geometry_msgs::Vector3 vel_value;
	
	cout<<"start\n";

	/* debug */
	geometry_msgs::Twist debug_rc;
	geometry_msgs::Twist debug_qp;	

	while(ros::ok()){
		if(serial_getc(&c) != -1) {
			imu_buf_push(c); 
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+'){
				if(imu_decode(imu.buf)==0){
	
					//debug
					debug_rc.linear.x = imu.acc[0];
					debug_rc.linear.y = imu.acc[1];
					debug_rc.linear.z = imu.gyrop[0];
					debug_rc.angular.x = roll_d;
					debug_rc.angular.y = pitch_d;
					debug_rc.angular.z = throttle_d;
					debug_rc_pub.publish(debug_rc);

					debug_qp.linear.x = acc_x;
					debug_qp.linear.y = acc_y;
					debug_qp.linear.z = acc_z;
					debug_qp.angular.x = acc_d[0];
					debug_qp.angular.y = acc_d[1];
					debug_qp.angular.z = acc_d[2];
					debug_qp_pub.publish(debug_qp);
			
					vel_value.x = vel[0];
					vel_value.y = vel[1];
					vel_value.z = vel[2];
					

					vel_pub.publish(vel_value);

					//send sol to uart
					send_pose_to_serial(roll_d,pitch_d,throttle_d,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
					//send_pose_to_serial(imu.acc[0],imu.acc[1],imu.gyrop[0],0.0,0.0,0.0,0.0,0.0,0.0,0.0);

				}
			}
		}
	}
}
