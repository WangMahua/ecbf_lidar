#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


#define PI 3.14159
using namespace std;
float pos[3],last_pos[3],vel[3];
double now_time,last_time;
float yaw = 0.0;
bool pose_init_flag=false;
float q_x,q_y,q_z,q_w;
geometry_msgs::Quaternion odom_quat;
nav_msgs::Path path;
ros::Publisher path_pub;
geometry_msgs::PoseStamped new_msg;

void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
		
		if (pose_init_flag == false){
			now_time = ros::Time::now().toSec();
			pos[0] = msg->pose.position.x;
			pos[1] = msg->pose.position.y;
			pos[2] = msg->pose.position.z;
			last_pos[0] = pos[0];
			last_pos[1] = pos[1];
			last_pos[2] = pos[2];
			last_time = now_time;
			pose_init_flag = true;	
		}
		else{
//			now_time = msg->header.stamp.toSec();
			now_time = ros::Time::now().toSec();
			pos[0] = msg->pose.position.x;
			pos[1] = msg->pose.position.y;
			pos[2] = msg->pose.position.z;
			vel[0] =(pos[0] - last_pos[0])/0.0083;
			vel[1] =(pos[1] - last_pos[1])/0.0083;
			vel[2] =(pos[2] - last_pos[2])/0.0083;
			last_pos[0] = pos[0];
			last_pos[1] = pos[1];
			last_pos[2] = pos[2];
			last_time = now_time;

		}
	odom_quat = msg->pose.orientation;
	q_x = msg->pose.orientation.x;
	q_y = msg->pose.orientation.y;
	q_z = msg->pose.orientation.z;
	q_w = msg->pose.orientation.w;

	//pub path info 
/*
	path.header = msg->header;
	path.poses.push_back(*msg);
	path_pub.publish(path);
		*/
}

int main(int argc ,char **argv){

  ros::init(argc,argv,"odometry");   
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
ros::Subscriber pos_sub = n.subscribe("/vrpn_client_node/MAV1/pose", 1, pos_callback);
  tf::TransformBroadcaster odom_broadcaster;
  path_pub = n.advertise<nav_msgs::Path>("ECBF_Path", 1);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100);
  while(n.ok()){

    // check for incoming messages
    current_time = ros::Time::now();
/*
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
*/
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-yaw);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pos[0];
    odom_trans.transform.translation.y = pos[1];
    odom_trans.transform.translation.z = pos[2];
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = pos[0];
    odom.pose.pose.position.y = pos[1];
    odom.pose.pose.position.z = pos[2];
/*
    odom.pose.pose.orientation.x = q_x;
    odom.pose.pose.orientation.y = q_y;
    odom.pose.pose.orientation.z = q_z;
    odom.pose.pose.orientation.w = q_w;
*/
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vel[0];
    odom.twist.twist.linear.y = vel[1];
    odom.twist.twist.linear.y = vel[2];
    //odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
	ros::spinOnce();
    r.sleep();
    
	
  }
}
