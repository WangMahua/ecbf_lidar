#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <serial.hpp>
#include <mutex>
#include <cmath>
#include <vector>
#include <string>
#include <iterator>

#include "osqp.h"
#include "QuadProg++/Array.hh"
#include "QuadProg++/QuadProg++.hh"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include "pcl_ros/point_cloud.h"

#include "tf/transform_listener.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

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

#define EQU_NUM 100

/*modify velocity*/
#define UPPER_V 4
#define LOWER_V -4
#define DEL_V 0.3

/*parameter od ECBF*/
#define SAFE_DIS 1
#define K1 2
#define K2 2

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
	
	/*calculate velocity*/
	double pos[3] = {0,0,0};
	double vel[3] = {0,0,0};
	double last_vel[3] = {0,0,0};
	double last_pos[3] = {0,0,0};
	double last_time = 0.0;
	bool pose_init_flag = false;

	

	ros::NodeHandle n;
	ros::Publisher debug_rc_pub,debug_qp_pub,debug_hz_pub,debug_vel_pub,debug_outputclound_pub;
	ros::Subscriber lidar_sub, pos_sub;
	ros::ServiceClient client;

	ros::Time now_vel_time ;
	ros::Time last_vel_time ;

	laser_geometry::LaserProjection projector;
	tf::TransformListener listener;

	pcl::PointCloud<pcl::PointXYZ> inputCloud;
	pcl::PointCloud<pcl::PointXYZ> outputCloud;

	

public:
	ecbf();
	void get_desire_rc_input(float,float,float,float,int);
	void get_desire_rc_input(rc_data);
	float bound_rc(float);
	void debug_pub();
	void debug_hz(float);
	void acc_cal();
	void rc_cal(float*);
	void get_sol(double*);
	void process();
	void reduce_pcl();
	float bound(float);
	void fix_vel();
	float fix_vel(float ,float );

	void qp_solve();

	void scan_callback(const sensor_msgs::LaserScan::ConstPtr&);
	void pos_callback(const geometry_msgs::PoseStamped::ConstPtr&);
};

ecbf::ecbf(){
	//ros 
	client = n.serviceClient<ecbf_lidar::qp>("qp");
	debug_rc_pub = n.advertise<ecbf_lidar::rc_compare>("rc_info", 1); 
	debug_qp_pub = n.advertise<ecbf_lidar::acc_compare>("qp_info", 1); 
	debug_hz_pub = n.advertise<std_msgs::Float32>("hz_info", 100); 
	debug_vel_pub = n.advertise<geometry_msgs::Vector3>("vel_info", 1); 
	debug_outputclound_pub = n.advertise<sensor_msgs::PointCloud2>("lidar_info", 1); 
	lidar_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &ecbf::scan_callback, this); 
	pos_sub = n.subscribe("/vrpn_client_node/MAV1/pose", 1, &ecbf::pos_callback, this); 
}



void ecbf::qp_solve(){
	double *p_ = pos;
	double *v_ = vel;
	float u[3] = {user_acc[0],user_acc[1],user_acc[2]}; // user input 

	/*get value from outputcloud*/
	// std::vector<geometry_msgs::Vector3> data;
	// for(int i =0;i<outputCloud.points.size();i++){
	// 	geometry_msgs::Vector3 temp;
	// 	temp.x = outputCloud.points[i].x;
	// 	temp.y = outputCloud.points[i].y;
	// 	temp.z = outputCloud.points[i].z;
	// 	data.push_back(temp);
	// }
	std::vector<std::vector<float>> data;
	for(int i =0;i<outputCloud.points.size();i++){
		std::vector<float> temp;
		temp.push_back(-1*outputCloud.points[i].x);
		temp.push_back(-1*outputCloud.points[i].y);
		temp.push_back(-1*outputCloud.points[i].z);
		//cout << "insert x:"<< outputCloud.points[i].x<< " y:"<< outputCloud.points[i].y<<" z:"<<outputCloud.points[i].z<<endl;
		data.push_back(temp);
	}
/*
	cout << "size of data:" <<data.size() <<endl;
	cout << "size of data:" <<data[0].size() <<endl;
*/
	/*change value into string*/
	quadprogpp::Matrix<double> G, CE, CI;
	quadprogpp::Vector<double> g0, ce0, ci0, x;
	int n, m, p;

	n = 3; // xyz 3-dimension
	G.resize(n, n);
	for (int i = 0; i < n; i++){
		for (int j = 0; j < n; j++){
			if(i==j) G[i][j] = 1;
			else G[i][j] = 0;
		}
	}
	g0.resize(n);
	for (int i = 0; i < n ; i++){
		g0[i] = - 0.5*u[i];
	}

	m = 0;
	CE.resize(n, m);
	ce0.resize(m);

	p = EQU_NUM;
	std::vector<float> square_sum;
	
	/*2(P-PCL)*/
	CI.resize(n, p);
	for (int i = 0; i < p; i++){
		float sum = 0.0;
		for (int j = 0; j < n; j++){
			CI[j][i]= data[i][j];
			sum+=pow(data[i][j],2);
		}
		sum -= pow(SAFE_DIS,2);
		square_sum.push_back(sum);
	}
	/*sum of vel suare*/
	float vel_square_sum = 0.0;
	for (int i =0;i<3;i++){
		vel_square_sum +=pow(vel[i],2);
	}

	ci0.resize(p);
	for (int i = 0; i < p ; i++){
		float temp_sum = 0.0;
		for(int j =0;j<3;j++){
			temp_sum+=2*data[i][j]*vel[j];
		}
		ci0[i] = 2*vel_square_sum + K1*square_sum[i]+K2*temp_sum;
	}
	x.resize(n);
	std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;
	std::cout << "u: " << "x: "<<u[0] << " y: "<<u[1]<< " z: "<<u[2]<< std::endl;
	std::cout << "x: " << x << std::endl;


	ecbf_acc[0] = x[0] ;
	ecbf_acc[1] = x[1] ;
	//ecbf_acc[2] = srv.response.ecbf_output[2] ;
	ecbf_acc[2] = user_rc_input[3]; //do not change throttle
}

float ecbf::bound(float v){
	if(v>UPPER_V){
		v = UPPER_V;
	}else if(v<LOWER_V){
		v = LOWER_V;
	}
	return v;
}

void ecbf::fix_vel(){
	for(int i=0;i<3;i++){
		vel[i]=fix_vel(vel[i],last_vel[i]);
	}
}

float ecbf::fix_vel(float now_vel,float old_vel){
	float new_vel = old_vel;
	float delta_vel = 0.0;
	delta_vel = now_vel-old_vel;
	if(abs(delta_vel)>DEL_V){
		if(delta_vel>0){
			new_vel +=DEL_V;
		}else{
			new_vel -=DEL_V;
		}
	}else{
		new_vel = now_vel;
	}
	new_vel = bound(new_vel);
	return new_vel;
	
}

void ecbf::pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
		if (pose_init_flag == false){
			now_vel_time =ros::Time::now();
			pos[0] = msg->pose.position.x;
			pos[1] = msg->pose.position.y;
			pos[2] = msg->pose.position.z;
			*last_pos = *pos;
			last_vel_time = now_vel_time;
			pose_init_flag = true;	
		}
		else{
			now_vel_time =ros::Time::now();
			float clustering_time = (now_vel_time - last_vel_time).toSec();

			pos[0] = msg->pose.position.x;
			pos[1] = msg->pose.position.y;
			pos[2] = msg->pose.position.z;
			vel[0] =(pos[0] - last_pos[0])/clustering_time;
			vel[1] =(pos[1] - last_pos[1])/clustering_time;
			vel[2] =(pos[2] - last_pos[2])/clustering_time;
			fix_vel();
			for(int i =0;i<3;i++){
				last_pos[i]=pos[i];
				last_vel[i]=vel[i];
				cout << vel[i] << endl;
			}
			last_vel_time = now_vel_time;
		}
}

void ecbf::reduce_pcl(){
	outputCloud.clear();
	int num = int(inputCloud.points.size()/EQU_NUM);
	for(int i =0;i<EQU_NUM;i++){
		int min_idx = 0;
		float min_ms = 1000.0;
		if(i!=EQU_NUM-1){
			for(int j = i*num ;j<(i+1)*num;j++){
				float ms = pow(inputCloud.points[j].x,2)+pow(inputCloud.points[j].y,2);
				if(ms<min_ms){
					min_idx=j;
					min_ms = ms;
				}
			}
		}else{
			for(int j = i*num ;j<inputCloud.points.size();j++){
				float ms = pow(inputCloud.points[j].x,2)+pow(inputCloud.points[j].y,2);
				if(ms<min_ms){
					min_idx=j;
					min_ms = ms;
				}
			}
		}
/*
		cout << "---"<< endl;
		cout << "ms: "<< min_ms << endl;
		cout << "j: "<< min_idx << endl;
		cout << "x: " << inputCloud.points[min_idx].x  << endl;
		cout << "y: " << inputCloud.points[min_idx].y  << endl;
		cout << "z: " << inputCloud.points[min_idx].z  << endl;
*/
		outputCloud.push_back(inputCloud.points[min_idx]);
	}
}

void ecbf::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	sensor_msgs::PointCloud2 input_pointcloud;

    projector.projectLaser(*msg, input_pointcloud); // laserscan to pcl2
    pcl::fromROSMsg(input_pointcloud, inputCloud);
	std::cout <<"input cloud size : " <<inputCloud.points.size()<<std::endl;
	reduce_pcl();
	std::cout <<"output cloud size : " <<outputCloud.points.size()<<std::endl;
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
	geometry_msgs::Vector3 debug_vel;
	sensor_msgs::PointCloud2 cloud;
	
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

	debug_vel.x = vel[0];
	debug_vel.y = vel[1];
	debug_vel.z = vel[2];

	debug_vel_pub.publish(debug_vel);

	toROSMsg(outputCloud, cloud);
	cloud.header.frame_id = "map";
	debug_outputclound_pub.publish(cloud);

}

void ecbf::debug_hz(float a){
	std_msgs::Float32 debug_hz;
	debug_hz.data = a;
	debug_hz_pub.publish(debug_hz);
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
/*
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
*/
		qp_solve();
		rc_cal(ecbf_rc);

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
	ros::NodeHandle k;
	
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
		
		/*
		cout<<"rc info in lidar:"<<endl;
		cout<<"rc.mode: "<<rc.mode<<"\n";
		cout<<"rc.roll: "<<rc.roll<<"\n";
		cout<<"rc.pitch: "<<rc.pitch<<"\n";
		cout<<"rc.yaw: "<<rc.yaw<<"\n";
		cout<<"rc.throttle: "<<rc.throttle<<"\n";
		cout<<"==="<<endl;
		*/
		ros::Time begin_time = ros::Time::now();

		ecbf_process.get_desire_rc_input(rc);
		
		ecbf_process.process();
		ecbf_process.get_sol(pub_to_controller);
		pub_to_controller[2]=rc.throttle;

		/* calculate time */
		ros::Time now_time = ros::Time::now();
		float clustering_time = 0.0 ;
		clustering_time = (now_time - begin_time).toSec();
		ecbf_process.debug_hz(clustering_time); // hz record 
		cout << "hz:"<< 1/clustering_time <<endl; //hz test
		
		loop2_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

int send_thread_entry(){
	ros::NodeHandle h;
	ros::Rate loop3_rate(100);

	while(ros::ok()){
		/* send data to uart */
		if(DEBUG_FLAG == 0){
			send_pose_to_serial(pub_to_controller[0],pub_to_controller[1],rc_value[3],0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		}else{
			send_pose_to_serial(rc_value[0],rc_value[1],rc_value[3],0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		}
		loop3_rate.sleep();
	}
}
