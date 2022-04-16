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
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
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

//#include "kalman_filter.h"
#include <Eigen/Dense>

/* This two value are the maximun and minimun value for rc */
#define upper_rc 35
#define lower_rc -35

/* modify velocity */
#define UPPER_V 4
#define LOWER_V -4
#define DEL_V 0.3

/* DEBUG_FLAG is a extra lock for user. if you just want to check */
/* the value calculate from ecbf and do not want to apply it to f-*/
/* light control board, you should set this flag as 1.            */
#define DEBUG_FLAG 0

/* EQU_NUM is the number of point you can get after reduce_pcl(), */
/* which related to the constraint number of qp solver.           */
#define EQU_NUM 100

/* parameter of ECBF */
#define SAFE_DIS 1
#define K1 2
#define K2 2

/* parameter of kalman filter */
/* n : Number of states       */
/* m : Number of measurements */
const int s_n = 2;
const int m_n = 2;

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

float pub_to_controller[3];
float per2thrust_coeff[6] = {930.56,-3969,4983.2,-1664.5,482.08,-7.7146};
float thrust2per_coeff[6] = {-1.11e-15,-3.88e-12,1.09e-8,-8.63e-6,3.62e-3,0};

/* global variable for uart data receive */
imu_t imu;
uint8_t rc_ch7;
float rc_value[4];
int rc_ecbf_mode;

std::mutex m_mutex;
std::mutex l_mutex;

class ecbf{
private:
	/* user_rc_input : roll pitch yaw throttle from user remote control */
	/* user_acc : user rc input translate to acc (for qp solve)         */
	/* ecbf_acc : new safe acc been calculated by qp solver             */
	/* ecbf_rc : roll pitch yaw throttle translate from ecbf_acc        */
	float user_rc_input[4] = {0.0,0.0,0.0,0.0}; 
	float user_acc[3] = {0.0,0.0,0.0}; 
	float ecbf_acc[3] = {0.0,0.0,0.0}; 
	float ecbf_rc[3] = {0.0,0.0,0.0}; 
	bool ecbf_mode = false;
	
	/* calculate velocity */
	double pos[3] = {0,0,0};
	double vel[3] = {0,0,0};
	double pos_kf[3] = {0,0,0};
	double vel_kf[3] = {0,0,0};
	double last_vel[3] = {0,0,0};
	double last_pos[3] = {0,0,0};
	double last_time = 0.0;
	bool pose_init_flag = false;

	/* check lidar initialize finish */
	bool lidar_init_flag = false;
	

	ros::NodeHandle n;
	ros::Publisher debug_rc_pub,debug_qp_pub,debug_hz_pub,debug_vel_pub,debug_vel_kf_pub,debug_pos_kf_pub,debug_outputclound_pub;
	ros::Subscriber lidar_sub, pos_sub;
	ros::ServiceClient client;

	ros::Time now_vel_time ;
	ros::Time last_vel_time ;

	laser_geometry::LaserProjection projector;
	tf::TransformListener listener;

	pcl::PointCloud<pcl::PointXYZ> inputCloud;
	pcl::PointCloud<pcl::PointXYZ> outputCloud;

	MatrixXd A,B,C,Q,R,P,I,K;
	VectorXd x_hat, x_hat_new,y;

	/* debug */
	ecbf_lidar::rc_compare debug_rc;
	ecbf_lidar::acc_compare debug_qp;
	geometry_msgs::Vector3 debug_vel;
	geometry_msgs::Vector3 debug_vel_kf;
	geometry_msgs::Vector3 debug_pos_kf;
	sensor_msgs::PointCloud2 cloud;
	
public:
	ecbf();
	void get_desire_rc_input(rc_data);
	void get_sol(float*); 
	void acc_cal();
	void rc_cal(float*);
	void process();

	/* debug */
	void debug_pub();
	void debug_hz(float);

	/* pcl */
	void reduce_pcl();

	/* limitation of vel and rc */
	float bound(float);
	float bound(float,float);
	float bound_rc(float);
	void fix_vel();
	float fix_vel(float,float);

	/* qp solver */
	void qp_solve();

	/* kalman filter to get vel */
	void vel_filter(float, float, float, int);

	/* ros callback function */
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr&);
	void pos_callback(const geometry_msgs::PoseStamped::ConstPtr&);
};

ecbf::ecbf():A(s_n,s_n),C(m_n,s_n),Q(s_n,s_n),R(m_n,m_n),P(s_n,s_n),x_hat(s_n),y(m_n),I(s_n,s_n),K(s_n,s_n){
	/* ros service and topic declare*/
	client = n.serviceClient<ecbf_lidar::qp>("qp");
	debug_rc_pub = n.advertise<ecbf_lidar::rc_compare>("rc_info", 1); 
	debug_qp_pub = n.advertise<ecbf_lidar::acc_compare>("qp_info", 1); 
	debug_hz_pub = n.advertise<std_msgs::Float32>("hz_info", 100); 
	debug_vel_pub = n.advertise<geometry_msgs::Vector3>("vel_info", 1); 
	debug_vel_kf_pub = n.advertise<geometry_msgs::Vector3>("vel_kf_info", 1); 
	debug_pos_kf_pub = n.advertise<geometry_msgs::Vector3>("pos_kf_info", 1); 
	debug_outputclound_pub = n.advertise<sensor_msgs::PointCloud2>("lidar_info", 1); 
	lidar_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 10, &ecbf::scan_callback, this); 
	pos_sub = n.subscribe("/vrpn_client_node/MAV1/pose", 10, &ecbf::pos_callback, this); 
	

	A << 1,0,0,1;
	C << 1,0,0,1;
	Q << 1e-6,0,0,1e-6;
	R << 1e-5,0,0,1e-5;
	P << 1e-4,0,0,1e-4;

	x_hat.setZero();
	y.setZero();
	K.setIdentity();
	I.setIdentity();
}

void ecbf::vel_filter(float new_p,float new_v,float dt,int d){
	y(0)=new_p;
	y(1)=new_v;
	VectorXd x_hat_new(2);
	A(0,1) = dt;
	x_hat_new = A * x_hat;
	P = A*P*A.transpose() + Q;
	K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
	x_hat_new += K * (y - C*x_hat_new);
	P = (I - K*C)*P;
	x_hat = x_hat_new;
}


void ecbf::qp_solve(){
	double *p_ = pos;
	double *v_ = vel;
	float u[3] = {user_acc[0],user_acc[1],user_acc[2]}; // user input 
	m_mutex.lock();
	/* change pcl point (from callback) to vector */
	std::vector<std::vector<float>> data;
	for(int i =0;i<outputCloud.points.size();i++){
		std::vector<float> temp;
		temp.push_back(-1*outputCloud.points[i].x);
		temp.push_back(-1*outputCloud.points[i].y);
		temp.push_back(-1*outputCloud.points[i].z);
		data.push_back(temp);
	}
	m_mutex.unlock();
	cout <<"pointcloud size: "<<data.size()<<endl;
	if (data.size()!=0){
		
		/* qp matrix give value */
		quadprogpp::Matrix<double> G, CE, CI;
		quadprogpp::Vector<double> g0, ce0, ci0, x;
		int n, m, p;

		n = 3; 
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

		p = data.size();
		std::vector<float> square_sum;
		
		/* 2(P-PCL) */
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
		/* sum of vel suare */
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

		/* We do not change throttle */
		ecbf_acc[0] = bound(x[0],10) ;
		ecbf_acc[1] = bound(x[1],10) ;
		//ecbf_acc[2] = bound(x[2],10) ;
		ecbf_acc[2] = user_rc_input[3]; 
	}else{

		static int qp_fail_cout = 0;
		cout << "weird:" << qp_fail_cout++<<endl;
		//for(int i =0;i<3;i++)
		//	ecbf_acc[i] = user_rc_input[i]; 	
		//ecbf_acc[2] = user_rc_input[3];
	}
}

float ecbf::bound(float v){
	if(v>UPPER_V){
		v = UPPER_V;
	}else if(v<LOWER_V){
		v = LOWER_V;
	}
	return v;
}
float ecbf::bound(float a,float l){
	
	float l_l = -1*l;
	if(a>l){
		a = l;
	}else if(a<l_l){
		a = l_l;
	}
	return a;
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
			for(int i =0;i<3;i++){
				last_pos[i]=pos[i];
				last_vel[i]=vel[i];
			}
			last_vel_time = now_vel_time;
			pose_init_flag = true;	
		}
		else{
			now_vel_time =ros::Time::now();
			float clustering_time = (now_vel_time - last_vel_time).toSec();

			pos[0] = msg->pose.position.x;
			pos[1] = msg->pose.position.y;
			pos[2] = msg->pose.position.z;

			for (int i =0;i<3;i++){
				vel[i] = (pos[i] - last_pos[i])/clustering_time;
				vel[i] = 1*(last_vel[i] + (vel[i] - last_vel[i])/clustering_time)+0*vel[i];
				vel[i] = fix_vel(vel[i],last_vel[i]);
				//vel_filter(pos[i],vel[i],clustering_time,i);
				//vel_kf[i]= x_hat[1];
				//pos_kf[i]= x_hat[0];

			}
			
			//fix_vel();
			//cout <<"vel" <<endl;
			for(int i =0;i<3;i++){
				last_pos[i]=pos[i];
				last_vel[i]=vel[i];
				//cout << vel[i] << endl;
			}
			//cout <<"vel from kf" <<endl;
			for(int i =0;i<3;i++){
				//cout << vel_kf[i] << endl;
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
		outputCloud.push_back(inputCloud.points[min_idx]);
	}
}

void ecbf::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	lidar_init_flag = true;
	ros::Time begin_time_pcl = ros::Time::now();
	sensor_msgs::PointCloud2 input_pointcloud;

    projector.projectLaser(*msg, input_pointcloud); // laserscan to pcl2
    pcl::fromROSMsg(input_pointcloud, inputCloud);


	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud_ptr=inputCloud.makeShared();
	//pcl::VoxelGrid<pcl::PointXYZ> sor_map;
	//sor_map.setInputCloud (cloud_ptr);
	//sor_map.setLeafSize (0.5f, 0.5f, 0.5f);
	//sor_map.filter (*cloud_ptr);
	//inputCloud=*cloud_ptr;

	//std::cout <<"input cloud size : " <<inputCloud.points.size()<<std::endl;
	l_mutex.lock();
	reduce_pcl();
	l_mutex.unlock();
	//std::cout <<"output cloud size : " <<outputCloud.points.size()<<std::endl;

	/* calculate time */
	ros::Time now_time_pcl = ros::Time::now();
	float clustering_time_pcl = 0.0 ;
	clustering_time_pcl = (now_time_pcl - begin_time_pcl).toSec();
	cout << "hz in pcl:"<< 1/clustering_time_pcl <<endl; //hz test
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

	debug_vel_kf.x = vel_kf[0];
	debug_vel_kf.y = vel_kf[1];
	debug_vel_kf.z = vel_kf[2];

	debug_vel_kf_pub.publish(debug_vel_kf);

	debug_pos_kf.x = pos_kf[0];
	debug_pos_kf.y = pos_kf[1];
	debug_pos_kf.z = pos_kf[2];

	debug_pos_kf_pub.publish(debug_pos_kf);

	//toROSMsg(outputCloud, cloud);
	//cloud.header.frame_id = "map";
	//debug_outputclound_pub.publish(cloud);

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

	if(ecbf_mode==true && lidar_init_flag==true){
		acc_cal(); 
		qp_solve();
		rc_cal(ecbf_rc);
		
	}else{
		for (int i = 0;i<3;i++) ecbf_rc[i] = user_rc_input[i]; //[0]:roll [1]:pitch [2]:throttle
		if(ecbf_mode!=true) cout<<"ecbf not trigger!"<<endl;
		else cout<<"lidar has yet to initialize."<<endl;
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

	throttle = 0;
	for(int i = 0;i<6;i++){
		throttle += thrust2per_coeff[5-i]*pow(force,i)*100;
	}

	cout << "roll_d:"<<roll<<'\n';
	cout << "pitch_d:"<<pitch<<'\n';
	cout << "force_d:"<<force<<"\t thrust:"<<throttle<<'\n';
	cout << "===\n";

	desire_rc[0] =  roll;
	desire_rc[1] =  pitch;
	desire_rc[2] =  throttle;
}

void ecbf::get_sol(float* d){
	d[0] = ecbf_rc[0];
	d[1] = ecbf_rc[1];
	d[2] = ecbf_rc[2];
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
    ros::Rate loop_rate_(800);
	char c;
	imu.buf_pos = 0;

	cout<<"start123\n";
	ecbf ecbf_process;
	uint8_t last_ecbf_mode = 0;
	ros::Time now_uart_time = ros::Time::now();
	ros::Time last_uart_time = ros::Time::now();


	while(ros::ok()){
		
		if(serial_getc(&c) != -1) {
			imu_buf_push(c); 
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+'){
				if(imu_decode(imu.buf)==0){
					now_uart_time = ros::Time::now();
					float clustering_uart_time = 0.0 ;
					clustering_uart_time = (now_uart_time - last_uart_time).toSec();
					cout << "hz for uart:"<< 1/clustering_uart_time <<endl; //hz test
					last_uart_time = now_uart_time;

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
					cout << "hz for qp solver:"<< 1/clustering_time <<endl; //hz test


					loop_rate_.sleep();


					if(DEBUG_FLAG == 0){
						send_pose_to_serial(pub_to_controller[0],pub_to_controller[1],rc_value[3],0.0,0.0,0.0,0.0,0.0,0.0,0.0);
					}else{
						send_pose_to_serial(rc_value[0],rc_value[1],rc_value[3],0.0,0.0,0.0,0.0,0.0,0.0,0.0);
					}	
					
				}
			}
		}
		//ros::spinOnce();
	}
	return 0;

}
int ros_thread_entry(){
   ros::Rate loop_rate(1600);

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;

}

