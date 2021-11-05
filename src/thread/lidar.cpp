#include "ros/ros.h"
#include <iostream>
#include <serial.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <lidar.h>
#include <mutex>
#include <cmath>
#include "osqp.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include "tf/transform_listener.h"
#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"


#include "osqp++.cc"
/*
#include "status.cc"
#include "escaping.h"
#include "escaping.cc"
#include "numbers.cc"
#include "str_cat.cc"
#include "string_view.cc"


#include "cordz_info.cc"
#include "cordz_handle.cc"
#include "cordz_functions.cc"
#include "cord_internal.cc"
#include "cord_rep_btree.cc"
#include "cord_rep_ring.cc"

#include "cord.h"
#include "cord.cc"


#include "mutex.cc"
#include "graphcycles.cc"

*/



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
using namespace osqp;
using namespace absl;



double roll_d,pitch_d,throttle_d;
float per2thrust_coeff[6] = {930.56,-3969,4983.2,-1664.5,482.08,-7.7146};
float thrust2per_coeff[6] = {-1.11e-15,-3.88e-12,1.09e-8,-8.63e-6,3.62e-3,0};
mutex imu_mutex;
imu_t imu;
int rc_ch7;
rc_data rc;




float bound_rc(float v){
	if(v>upper_rc){
		v = upper_rc;
	}else if(v<lower_rc){
		v = lower_rc;
	}
	return v;
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

	memcpy(&rc.roll, &buf[2], sizeof(float)); //in ned coordinate system
	memcpy(&rc.pitch, &buf[6], sizeof(float));
	memcpy(&rc.yaw, &buf[10], sizeof(float));

	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&rc.throttle, &buf[14], sizeof(float));
	memcpy(&rc.mode, &buf[18], sizeof(int));
	
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
float* acc_cal(float* acc){
	float _roll,_pitch,_yaw,_throttle;
	_roll = -rc.roll*M_PI/180.0;
	_pitch = -rc.pitch*M_PI/180.0;
	_yaw = -rc.yaw*M_PI/180.0;
	_throttle = rc.throttle;
	
	float force = 0 ;
	for(int i = 0;i<6;i++){
		force += per2thrust_coeff[5-i]*pow(_throttle*0.01,i);
	}
	force = force/1000*4*GRAVITY;
	force = force<0?0:force;

	cout << "rc mode:" << rc.mode << '\n';
	cout << "height:" << pos[2] << '\n';
	acc[0] = GRAVITY*(_roll*cos(_yaw)+_pitch*sin(_yaw));
	acc[1] = GRAVITY*(_roll*sin(_yaw)-_pitch*cos(_yaw));
	acc[2] = force/MASS-GRAVITY;
	return acc;
}

float* qp_solve(float* acc){
    float px = pos[0];
    float py = pos[1];
    float pz = pos[2];
    float vx = vel[0];
    float vy = vel[1];
    float vz = vel[2];

    // Load problem data
    c_float P_x[3] = {1.0, 1.0, 1.0, };
    c_int P_nnz = 3;
    c_int P_i[3] = {0, 1, 2, };
    c_int P_p[4] = {0, 1, 2, 3,};
    c_float q[3] = {-acc[0], -acc[1], -acc[2]};

	#if PATH_MODE == 0
		c_float A_x[3] = {1.0, 1.0, 1.0, };
		c_int A_nnz = 3;
		c_int A_i[3] = {0, 1, 2, };
		c_int A_p[4] = {0, 1, 2, 3, };
		c_float l[3] = {-K1*(px-X_LOWER_BOUND)-K2*(vx), -K1*(py-Y_LOWER_BOUND)-K2*(vy), -K1*(pz-Z_LOWER_BOUND)-K2*(vz), };
		c_float u[3] = {K1*(X_UPPER_BOUND-px)-K2*(vx),K1*(Y_UPPER_BOUND-py)-K2*(vy), K1*(Z_UPPER_BOUND-pz)-K2*(vz), };
		c_int n = 3;
		c_int m = 3;
	#elif PATH_MODE == 1
		c_float A_x[3] = {2.0*px, 2.0*py, 1.0, };
		c_int A_nnz = 3;
		c_int A_i[3] = {0, 0, 1, };
		c_int A_p[4] = {0, 1, 2, 3, };
		c_float l[2] = {-K1*(px*px+py*py-R_MIN*R_MIN)-K2*(2*vx*px+2*vy*py) -2*(vx*vx+vy*vy), -K1*(pz-Z_LOWER_BOUND)-K2*(vz), };
		c_float u[2] = {K1*(R_MAX*R_MAX-px*px-py*py)-K2*(2*vx*px+2*vy*py) -2*(vx*vx+vy*vy), K1*(Z_UPPER_BOUND-pz)-K2*(vz), };
		c_int n = 3;
		c_int m = 2;
	#endif

    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }

    // Define solver settings as default
    if (settings) {
        osqp_set_default_settings(settings);
        settings->alpha = 1.0; // Change alpha parameter
		settings->verbose = false;
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);

    // Cleanup
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);
	acc[0] = work->solution->x[0];
	acc[1] = work->solution->x[1];
	acc[2] = work->solution->x[2];

	return acc;
}

vector<geometry_msgs::Point32> pcl_resize(sensor_msgs::PointCloud  origin_pcl){
	vector<geometry_msgs::Point32> a;
	vector<geometry_msgs::Point32>::iterator it_i;
	
	for(it_i=origin_pcl.points.begin(); it_i!=origin_pcl.points.end(); ++it_i){
		if(abs(it_i->x)<R_SAFE && abs(it_i->y)<R_SAFE){
			a.push_back(*it_i);
		}
	}
	cout << a.size()<< endl;
	return a;
}

void fill_P_Matrix(Eigen::SparseMatrix<double>* qp_P) {
    const Eigen::Triplet<double> kTripletsP[] = {{0, 0, 1.0}, {1, 1, 1.0}, {2, 2, 1.0},};
    qp_P->setFromTriplets(std::begin(kTripletsP),std::end(kTripletsP));
}

void fill_A_Matrix(vector<geometry_msgs::Point32> input_pcl_vector,Eigen::SparseMatrix<double>* qp_A) {
	vector<Eigen::Triplet<float>> kTripletsP;
	for(int i=0;i<input_pcl_vector.size();i++){
		kTripletsP.push_back({i,0,input_pcl_vector[i].x});
		kTripletsP.push_back({i,1,input_pcl_vector[i].y});
		kTripletsP.push_back({i,2,input_pcl_vector[i].z});
	}
	qp_A->setFromTriplets(std::begin(kTripletsP),
                                       std::end(kTripletsP));
}

Eigen::VectorXd fill_lower_bound(vector<geometry_msgs::Point32> input_pcl_vector,Eigen::VectorXd qp_l) {
	qp_l.resize(input_pcl_vector.size());
    float vx = vel[0];
    float vy = vel[1];
    float vz = vel[2];
	for(int i=0;i<input_pcl_vector.size();i++){
		float px = input_pcl_vector[i].x;
		float py = input_pcl_vector[i].y;
		float pz = input_pcl_vector[i].z;
		float lower_bound_value = -2*(vx*vx+vy*vy+vz*vz)-K1*(px*px+py*py+pz*pz-R_SAFE*R_SAFE)-2*K2*(px*vx+py*vy+pz*vz) ;
		qp_l(i) = lower_bound_value;
	}
	return qp_l;
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laser_geometry::LaserProjection projector_;
	tf::TransformListener listener;
	sensor_msgs::PointCloud2 cloud;
	sensor_msgs::PointCloud out_cloud;

	projector_.transformLaserScanToPointCloud("laser",*msg, cloud,listener);
	sensor_msgs::convertPointCloud2ToPointCloud(cloud,out_cloud);
	cout << "origin size:" << out_cloud.points.size() << endl;

	//decrease point number
	vector<geometry_msgs::Point32> resize_cloud;
	resize_cloud = pcl_resize(out_cloud);
	cout<<resize_cloud[0].x <<endl;

	float qp_acc[3]={0.0,0.0,0.0};
	float origin_acc[3]={0.0,0.0,0.0};
	acc_cal(origin_acc);
	*qp_acc = *origin_acc;

	//QP solver
	const double kInfinity = std::numeric_limits<double>::infinity();
	Eigen::SparseMatrix<double> objective_matrix(3, 3);
	Eigen::SparseMatrix<double> constraint_matrix(resize_cloud.size(), 3);
	Eigen::VectorXd lower_bounds_vector;

	fill_P_Matrix(&objective_matrix);
	fill_A_Matrix(resize_cloud,&constraint_matrix);
	OsqpInstance instance;
	instance.objective_matrix = objective_matrix;
	instance.constraint_matrix = constraint_matrix;
	instance.objective_vector.resize(3);
	instance.objective_vector << origin_acc[0], origin_acc[1],origin_acc[2];
	instance.lower_bounds  = fill_lower_bound(resize_cloud,lower_bounds_vector);
	instance.upper_bounds.resize(1);
	instance.upper_bounds << kInfinity;


	OsqpSolver solver;
	OsqpSettings settings;
	// Edit settings if appropriate.
	auto status = solver.Init(instance, settings);
	// Assuming status.ok().
	OsqpExitCode exit_code = solver.Solve();
	// Assuming exit_code == OsqpExitCode::kOptimal.
	double optimal_objective = solver.objective_value();
	Eigen::VectorXd optimal_solution = solver.primal_solution();

}

int lidar_thread_entry(){
	char c;
	imu.buf_pos = 0;
	
	float rc_throttle,rc_roll,rc_pitch,rc_yaw,force,force_d;
	float acc[3]={0.0,0.0,0.0};
	float acc_d[3]={0.0,0.0,0.0};

	ros::NodeHandle n;
	ros::Publisher qp_pub = n.advertise<geometry_msgs::Twist>("qp", 1); 
	ros::Publisher debug_rc_pub = n.advertise<geometry_msgs::Twist>("rc_info", 1); 
	ros::Publisher debug_qp_pub = n.advertise<geometry_msgs::Twist>("qp_info", 1); 
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3>("vel_info", 1); 
	ros::Subscriber lidar_sub = n.subscribe("/scan",1,lidar_callback);
	/* debug */
	geometry_msgs::Twist debug_rc;
	geometry_msgs::Twist debug_qp;	
	geometry_msgs::Vector3 vel_value;

	cout<<"start\n";

	while(ros::ok()){
		if(serial_getc(&c) != -1) {
			imu_buf_push(c); 
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+')
			{
				/*
				for(int i =0;i<IMU_SERIAL_MSG_SIZE;i++)
					cout << imu.buf[i];
				cout<<endl;
				*/
				if(imu_decode(imu.buf)==0)
				{
					rc_roll = -rc.roll*M_PI/180.0;
					rc_pitch = -rc.pitch*M_PI/180.0;
					rc_yaw = -rc.yaw*M_PI/180.0;
					rc_throttle = rc.throttle;
					
					force = 0 ;
					for(int i = 0;i<6;i++){
						force += per2thrust_coeff[5-i]*pow(rc_throttle*0.01,i);
					}
					force = force/1000*4*GRAVITY;
					force = force<0?0:force;

					cout << "rc mode:" << rc.mode << '\n';
					cout << "height:" << pos[2] << '\n';

					if(rc.mode>1.1 && pos[2]>ECBF_THESHOLD){ /* rc mode change & height > threshold*/
						acc[0] = GRAVITY*(rc_roll*cos(rc_yaw)+rc_pitch*sin(rc_yaw));
						acc[1] = GRAVITY*(rc_roll*sin(rc_yaw)-rc_pitch*cos(rc_yaw));
						acc[2] = force/MASS-GRAVITY;

						acc_d[0] = acc[0];
						acc_d[1] = acc[1];
						acc_d[2] = acc[2];

						cout << "acc[0]:"<<acc_d[0]<<'\n';
						cout << "acc[1]:"<<acc_d[1]<<'\n';
						cout << "acc[2]:"<<acc_d[2]<<'\n';

						qp_solve(acc_d);
						cout << "qp acc[0]:"<<acc_d[0]<<'\n';
						cout << "qp acc[1]:"<<acc_d[1]<<'\n';
						cout << "qp acc[2]:"<<acc_d[2]<<'\n';

						roll_d = -((cos(rc_yaw)*acc_d[0]+sin(rc_yaw)*acc_d[1])/GRAVITY*180.0/M_PI);
						pitch_d = -((-cos(rc_yaw)*acc_d[1]+sin(rc_yaw)*acc_d[0])/GRAVITY*180.0/M_PI);
						force_d = MASS*(acc_d[2] + GRAVITY);
						force_d = force_d /4 *1000 /9.81;
						roll_d = bound_rc(roll_d);
						pitch_d = bound_rc(pitch_d);

						cout << "roll_d:"<<roll_d<<'\n';
						cout << "pitch_d:"<<pitch_d<<'\n';

						throttle_d = 0;
						for(int i = 0;i<6;i++){
							throttle_d += thrust2per_coeff[5-i]*pow(force_d,i)*100;
						}

						cout << "force_d:"<<force_d<<"\t thrust:"<<throttle_d<<'\n';
						cout << "===\n";

					}else{
					    cout <<"not triggered!\n";	
						roll_d = rc.roll;
						pitch_d = rc.pitch;
						throttle_d = rc.throttle;
						*acc = *acc_d;
					}
	
				
					//debug
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
					debug_qp_pub.publish(debug_qp);
			
					vel_value.x = vel[0];
					vel_value.y = vel[1];
					vel_value.z = vel[2];
					

					vel_pub.publish(vel_value);

					//send sol to uart
					//send_pose_to_serial(roll_d,pitch_d,throttle_d,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
					//send_pose_to_serial(imu.acc[0],imu.acc[1],imu.gyrop[0],0.0,0.0,0.0,0.0,0.0,0.0,0.0);


				}
			}
		}
		
	}
	return 0;
}
