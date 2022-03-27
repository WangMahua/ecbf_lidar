#include "ros/ros.h"
#include <iostream>
#include <serial.hpp>
#include <mutex>
#include <cmath>

#include "ecbf_lidar/qp.h"
#include "ecbf_lidar/acc_compare.h"
#include "ecbf_lidar/rc_compare.h"
#include "main_thread.h"

/* This two value is the maximun and minimun value for rc */
#define upper_rc 35
#define lower_rc -35

/* DEBUG_FLAG is a extra lock for user. if you just want to check */
/* the value calculate from ecbf and do not want to apply it to f-*/
/* light control board, you should set this flag as 1.            */
#define DEBUG_FLAG 0

using namespace std;

double pub_to_controller[3];
float per2thrust_coeff[6] = {930.56,-3969,4983.2,-1664.5,482.08,-7.7146};
float thrust2per_coeff[6] = {-1.11e-15,-3.88e-12,1.09e-8,-8.63e-6,3.62e-3,0};
imu_t imu;
uint8_t rc_ch7;
std::mutex l_mutex;
std::mutex g_mutex;

float rc_value[4];
int rc_ecbf_mode;

class ecbf{
private:
	float user_rc_input[4] = {0.0,0.0,0.0,0.0}; //user rc input (roll pitch yaw throttle)
	float user_acc[3] = {0.0,0.0,0.0}; // user rc input change to acc (yaw angle is constant)
	float ecbf_acc[3] = {0.0,0.0,0.0}; // new safe 
	float ecbf_rc[3] = {0.0,0.0,0.0}; //roll pitch throttle
	bool ecbf_mode = false;

	ros::NodeHandle n;
	ros::Publisher debug_rc_pub,debug_qp_pub;
	ros::ServiceClient client;

public:
	ecbf();
	void get_desire_rc_input(float,float,float,float,int);
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
	debug_rc_pub = n.advertise<ecbf_lidar::rc_compare>("rc_info", 1); 
	debug_qp_pub = n.advertise<ecbf_lidar::acc_compare>("qp_info", 1); 
}
void ecbf::get_desire_rc_input(float rc_roll,float rc_pitch,\
	float rc_yaw,float rc_throttle,int rc_mode){
	user_rc_input[0] = rc_roll;
	user_rc_input[1] = rc_pitch;
	user_rc_input[2] = rc_yaw;
	user_rc_input[3] = rc_throttle;

	if(rc_mode>1.1){
		ecbf_mode = true ; 
	}else{
		ecbf_mode = false ; 
	}
}
void ecbf::get_desire_rc_input(rc_data rc_){
	user_rc_input[0] = rc_.roll;
	user_rc_input[1] = rc_.pitch;
	user_rc_input[2] = rc_.yaw;
	user_rc_input[3] = rc_.throttle;
	if(rc_.mode>1.1){
		ecbf_mode = true ; 
	}else{
		ecbf_mode = false ; 
	}
}

void ecbf::debug_pub(){ 
	/* debug */
	ecbf_lidar::rc_compare debug_rc;
	ecbf_lidar::acc_compare debug_qp;
	
	debug_rc.origin_roll = user_rc_input[0];
	debug_rc.origin_pitch = user_rc_input[1];
	debug_rc.origin_throttle = user_rc_input[3];
	debug_rc.ecbf_roll = ecbf_rc[0];
	debug_rc.ecbf_pitch = ecbf_rc[1];
	debug_rc.ecbf_throttle = ecbf_rc[2];

	debug_rc_pub.publish(debug_rc);

	debug_qp.origin_acc_x = user_acc[0];
	debug_qp.origin_acc_y = user_acc[1];
	debug_qp.origin_acc_z = user_acc[2];
	debug_qp.ecbf_acc_x = ecbf_acc[0];
	debug_qp.ecbf_acc_y = ecbf_acc[1];
	debug_qp.ecbf_acc_z = ecbf_acc[2];
	
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
			srv.request.desire_input.push_back(user_acc[i]);
		}

		if (client.call(srv)){
			cout << "ecbf_acc[0] : " <<srv.response.ecbf_output[0] << endl;
			cout << "ecbf_acc[1] : " <<srv.response.ecbf_output[1] << endl;
			cout << "ecbf_acc[2] : " <<srv.response.ecbf_output[2] << endl;

			ecbf_acc[0] = srv.response.ecbf_output[0] ;
			ecbf_acc[1] = srv.response.ecbf_output[1] ;
			//ecbf_acc[2] = srv.response.ecbf_output[2] ;
			ecbf_acc[2] = user_rc_input[3]; //do not change throttle
			rc_cal(ecbf_rc);

		}else{
			ROS_ERROR("Failed to calc");
			ecbf_rc[0] = user_rc_input[0] ; //roll
			ecbf_rc[1] = user_rc_input[1] ; //pitch
			ecbf_rc[2] = user_rc_input[3] ; //throttle
		}

	}else{
		cout<< "ecbf not trigger!\n";
		ecbf_rc[0] = user_rc_input[0] ; //roll
		ecbf_rc[1] = user_rc_input[1] ; //pitch
		ecbf_rc[2] = user_rc_input[3] ; //throttle
	}
	debug_pub();
}

void ecbf::acc_cal(){
	float _roll,_pitch,_yaw,_throttle;
	_roll = -user_rc_input[0]*M_PI/180.0;
	_pitch = -user_rc_input[1]*M_PI/180.0;
	_yaw = -user_rc_input[2]*M_PI/180.0; //set 0
	_throttle = user_rc_input[4];

	float force = 0 ;
	for(int i = 0;i<6;i++){
		force += per2thrust_coeff[5-i]*pow(_throttle*0.01,i);
	}
	force = force/1000*4*GRAVITY;
	force = force<0?0:force;

	user_acc[0] = GRAVITY*(_roll*cos(_yaw)+_pitch*sin(_yaw));
	user_acc[1] = GRAVITY*(_roll*sin(_yaw)-_pitch*cos(_yaw));
	user_acc[2] = force/MASS-GRAVITY;
}

void ecbf::rc_cal(float* desire_rc){
	float rc_yaw = user_rc_input[2];
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
	d[0] = ecbf_rc[0];
	d[1] = ecbf_rc[1];
	d[2] = ecbf_rc[2];
}

//** uart **//
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
int uart_thread_entry(){
	ros::NodeHandle n;
    	ros::Rate loop_rate(400);
	char c;
	imu.buf_pos = 0;

	cout<<"start\n";
	while(ros::ok()){
		
		if(serial_getc(&c) != -1) {
			imu_buf_push(c); 
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+'){
				if(imu_decode(imu.buf)==0){
/*
					cout<<"rc info in uart:"<<endl;
					cout<<"rc.mode: "<<rc_ecbf_mode<<"\n";
					cout<<"rc.roll: "<<rc_value[0]<<"\n";
					cout<<"rc.pitch: "<<rc_value[1]<<"\n";
					cout<<"rc.yaw: "<<rc_value[2]<<"\n";
					cout<<"rc.throttle: "<<rc_value[3]<<"\n";
					cout<<"==="<<endl;
*/
					loop_rate.sleep();			
				}
			}
		}
		
	}
	return 0;

}


int lidar_thread_entry(){
	char c;
	imu.buf_pos = 0;
	ecbf ecbf_process;
	uint8_t last_ecbf_mode = 0;
	ros::Rate loop2_rate(100);

	while(ros::ok()){
		
		if(rc_ecbf_mode>2.1){
			rc_ecbf_mode = last_ecbf_mode;
		}
		rc_data rc = { .roll = rc_value[0], \
						.pitch = rc_value[1], \
						.yaw = rc_value[2], \
						.throttle = rc_value[3], \
						.mode = rc_ecbf_mode };
		last_ecbf_mode = rc_ecbf_mode;
		

		cout<<"rc info in lidar:"<<endl;
		cout<<"rc.mode: "<<rc.mode<<"\n";
		cout<<"rc.roll: "<<rc.roll<<"\n";
		cout<<"rc.pitch: "<<rc.pitch<<"\n";
		cout<<"rc.yaw: "<<rc.yaw<<"\n";
		cout<<"rc.throttle: "<<rc.throttle<<"\n";
		cout<<"==="<<endl;
		
		ros::Time begin_time = ros::Time::now();

		ecbf_process.get_desire_rc_input(rc);
		
		ecbf_process.process();
		ecbf_process.get_sol(pub_to_controller);
		pub_to_controller[2]=rc.throttle;

		/* calculate time */
		ros::Time now_time = ros::Time::now();
		float clustering_time = 0.0 ;
		clustering_time = (now_time - begin_time).toSec();
		cout << "hz:"<< 1/clustering_time <<endl; //hz test
		
		/* send data to uart */
		if(DEBUG_FLAG == 0){
			send_pose_to_serial(pub_to_controller[0],pub_to_controller[1],rc.throttle,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		}else{
			send_pose_to_serial(rc.roll,rc.pitch,rc.throttle,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		}
		loop2_rate.sleep();
	}
	return 0;
}
