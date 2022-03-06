
#include "ros/ros.h"
#include <iostream>
#include <serial.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <lidar.h>
#include <mutex>
#include <cmath>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include "tf/transform_listener.h"
#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ecbf_lidar/qp.h"


#include "vel.h"
#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "vector"


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

//LiDAR
#define R_SAFE 0.3

using namespace std;

double roll_d,pitch_d,throttle_d;
double pub_to_controller[3];
float per2thrust_coeff[6] = {930.56,-3969,4983.2,-1664.5,482.08,-7.7146};
float thrust2per_coeff[6] = {-1.11e-15,-3.88e-12,1.09e-8,-8.63e-6,3.62e-3,0};
mutex imu_mutex;
imu_t imu;
int rc_ch7;
rc_data rc;
int m;

class ecbf{
private:
	float desired_rc_input[4] = {0.0,0.0,0.0,0.0}; //roll pitch yaw throttle
	float desired_acc[3] = {0.0,0.0,0.0};
	float ecbf_acc[3] = {0.0,0.0,0.0};
	float rc_modified[3] = {0.0,0.0,0.0}; //roll pitch throttle
	bool ecbf_mode = false;

	ros::NodeHandle n;
	ros::Publisher debug_rc_pub,debug_qp_pub;
	ros::ServiceClient client;

public:
	ecbf();
	void get_desire_rc_input(float,float,float,float,float);
	void get_desire_rc_input(rc_data);
	float bound_rc(float);
	void debug_pub();
	void acc_cal();
	void rc_cal(float*);
	void get_sol(double*);
	void process();

};
ecbf::ecbf(){
	//ros 
	client = n.serviceClient<ecbf_lidar::qp>("qp");
	debug_rc_pub = n.advertise<geometry_msgs::Twist>("rc_info", 1); 
	debug_qp_pub = n.advertise<geometry_msgs::Twist>("qp_info", 1); 
}
void ecbf::get_desire_rc_input(float rc_roll,float rc_pitch,\
	float rc_yaw,float rc_throttle,float rc_mode){
	desired_rc_input[0] = rc_roll;
	desired_rc_input[1] = rc_pitch;
	desired_rc_input[2] = rc_yaw;
	desired_rc_input[3] = rc_throttle;
	if(rc_mode>1.1){
		ecbf_mode = true ; 
	}else{
		ecbf_mode = false ; 
	}
}
void ecbf::get_desire_rc_input(rc_data rc_){
	desired_rc_input[0] = rc_.roll;
	desired_rc_input[1] = rc_.pitch;
	desired_rc_input[2] = rc_.yaw;
	desired_rc_input[3] = rc_.throttle;
	if(rc_.mode>1.1){
		ecbf_mode = true ; 
	}else{
		ecbf_mode = false ; 
	}
}
void ecbf::debug_pub(){
	/* debug */
	geometry_msgs::Twist debug_rc;
	geometry_msgs::Twist debug_qp;
/*
	debug_rc.linear.x = rc.roll;
	debug_rc.linear.y = rc.pitch;
	debug_rc.linear.z = rc.throttle;
	debug_rc.angular.x = roll_d;
	debug_rc.angular.y = pitch_d;
	debug_rc.angular.z = throttle_d;
	debug_rc_pub.publish(debug_rc);
	debug_qp.linear.x = acc[0];
	debug_qp.linear.y = acc[1];
	debug_qp.linear.z = acc[2];
	debug_qp.angular.x = acc_d[0];
	debug_qp.angular.y = acc_d[1];
	debug_qp.angular.z = acc_d[2];
	*/
	debug_qp_pub.publish(debug_qp);
}

float ecbf::bound_rc(float v){
	if(v>upper_rc){
		v = upper_rc;
	}else if(v<lower_rc){
		v = lower_rc;
	}
	return v;
}

void ecbf::process(){

	if(ecbf_mode==true){
		/*change rc to qp*/
		acc_cal();
		ecbf_lidar::qp srv;
		for(int i =0;i<3;i++){
			srv.request.desire_input.push_back(desired_acc[i]);
		}

		if (client.call(srv)){
			cout << "ecbf_acc[0] : " <<srv.response.ecbf_output[0] << endl;
			cout << "ecbf_acc[1] : " <<srv.response.ecbf_output[1] << endl;
			cout << "ecbf_acc[2] : " <<srv.response.ecbf_output[2] << endl;

			ecbf_acc[0] = srv.response.ecbf_output[0] ;
			ecbf_acc[1] = srv.response.ecbf_output[1] ;
			ecbf_acc[2] = srv.response.ecbf_output[2] ;
			rc_cal(rc_modified);

		}else{
			ROS_ERROR("Failed to calc");
			rc_modified[0] = desired_rc_input[0] ; //roll
			rc_modified[1] = desired_rc_input[1] ; //pitch
			rc_modified[2] = desired_rc_input[3] ; //throttle
		}

	}else{
		rc_modified[0] = desired_rc_input[0] ; //roll
		rc_modified[1] = desired_rc_input[1] ; //pitch
		rc_modified[2] = desired_rc_input[3] ; //throttle
	}
}

void ecbf::acc_cal(){
	float _roll,_pitch,_yaw,_throttle;
	_roll = -desired_rc_input[0]*M_PI/180.0;
	_pitch = -desired_rc_input[1]*M_PI/180.0;
	_yaw = -desired_rc_input[2]*M_PI/180.0;
	_throttle = desired_rc_input[4];

	float force = 0 ;
	for(int i = 0;i<6;i++){
		force += per2thrust_coeff[5-i]*pow(_throttle*0.01,i);
	}
	force = force/1000*4*GRAVITY;
	force = force<0?0:force;

	desired_acc[0] = GRAVITY*(_roll*cos(_yaw)+_pitch*sin(_yaw));
	desired_acc[1] = GRAVITY*(_roll*sin(_yaw)-_pitch*cos(_yaw));
	desired_acc[2] = force/MASS-GRAVITY;
}
void ecbf::rc_cal(float* desire_rc){
	float rc_yaw = desired_rc_input[2];
	float roll,pitch,force,throttle;

	roll = -((cos(rc_yaw)*ecbf_acc[0]+sin(rc_yaw)*ecbf_acc[1])/GRAVITY*180.0/M_PI);
	pitch = -((-cos(rc_yaw)*ecbf_acc[1]+sin(rc_yaw)*ecbf_acc[0])/GRAVITY*180.0/M_PI);
	force = MASS*(ecbf_acc[2] + GRAVITY);
	force = force /4 *1000 /9.81;
	roll = bound_rc(roll);
	pitch = bound_rc(pitch);

	cout << "roll_d:"<<roll<<'\n';
	cout << "pitch_d:"<<pitch<<'\n';

	throttle = 0;
	for(int i = 0;i<6;i++){
		throttle += thrust2per_coeff[5-i]*pow(force,i)*100;
	}

	cout << "force_d:"<<force<<"\t thrust:"<<throttle<<'\n';
	cout << "===\n";

	desire_rc[0] =  roll;
	desire_rc[1] =  pitch;
	desire_rc[2] =  throttle;
}

void ecbf::get_sol(double* d){
	d[0] = rc_modified[0];
	d[1] = rc_modified[1];
	d[2] = rc_modified[2];
}

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

	memcpy(&roll, &buf[2], sizeof(float)); //in ned coordinate system
	memcpy(&pitch, &buf[6], sizeof(float));
	memcpy(&yaw, &buf[10], sizeof(float));

	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&throttle, &buf[14], sizeof(float));
	memcpy(&rc_ch7, &buf[18], sizeof(int));
	//memcpy(&imu.gyrop[0], &buf[22], sizeof(float));

	rc.roll = roll; //east
	rc.pitch = pitch; //north
	rc.yaw = yaw; //up
	rc.throttle = throttle;
	rc.mode = rc_ch7;
	
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


int lidar_thread_debug_entry(){
	char c;
	imu.buf_pos = 0;

	ecbf ecbf_process;

	cout<<"start\n";
	while(ros::ok()){
		if(serial_getc(&c) != -1) {
			imu_buf_push(c); 
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+'){
				if(imu_decode(imu.buf)==0){
					cout<<"rc info:"<<endl;
					cout<<"rc.mode: "<<rc.mode<<"\n";
					cout<<"rc.roll: "<<rc.roll<<"\n";
					cout<<"rc.pitch: "<<rc.pitch<<"\n";
					cout<<"rc.yaw: "<<rc.yaw<<"\n";
					cout<<"rc.throttle: "<<rc.throttle<<"\n";
					cout<<"==="<<endl;
					//ecbf_process.get_desire_rc_input(rc.roll,rc.pitch,rc.yaw,rc.throttle,rc.mode);
					ecbf_process.get_desire_rc_input(rc);
/*
					ecbf_process.process();

					ecbf_process.get_sol(pub_to_controller);
		*/
					//send sol to uart
					//send_pose_to_serial(roll_d,pitch_d,throttle_d,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
					//send_pose_to_serial(imu.acc[0],imu.acc[1],imu.gyrop[0],0.0,0.0,0.0,0.0,0.0,0.0,0.0);
				}
			}
		}
	}
	return 0;
}


