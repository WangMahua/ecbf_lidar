#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <algorithm> // std::min_element
#include <iterator>  // std::begin, std::end
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include "pcl_ros/point_cloud.h"

#include "tf/transform_listener.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "laser_geometry/laser_geometry.h"


#define EQU_NUM 10

laser_geometry::LaserProjection projector;
/*
void reduce_pcl(pcl::PointCloud<pcl::PointXYZ> *input){
	pcl::PointCloud<pcl::PointXYZ> outCloud;
	int num = int(input.points.size()/EQU_NUM);
	for(int i =0;i<EQU_NUM;i++){
		std::vector<float> vect(num,1000.0);
		if(i!=EQU_NUM-1){
			for(int j = i*num ;j<(i+1)*num;j++){
				float ms = pow(input.points[j].x,2)+pow(input.points[j].y,2);
				std::cout<<"ms:"<<ms<<std::endl;
				vect.push_back(ms);
			}
		}else{
			for(int j = i*num ;j<input.points.size();j++){
				float ms = pow(input.points[j].x,2)+pow(input.points[j].y,2);
				vect.push_back(ms);
			}
		}

		auto it = std::min_element(std::begin(vect), std::end(vect));
		
		std::cout << "index of smallest element: " << *it<< std::endl;
		std::cout << "value: " << vect[std::distance(std::begin(vect), it)] << std::endl;
		
		outCloud.push_back(input[vect[std::distance(std::begin(vect), it)]]);
	}
	std::cout <<"size : " <<outCloud.points.size()<<std::endl;
	*input = *outCloud;
}

void reduce_pcl(){
	pcl::PointCloud<pcl::PointXYZ> outCloud;
	int num = int(input->points.size()/EQU_NUM);
	for(int i =0;i<EQU_NUM;i++){
		std::vector<float> vect(num,1000.0);
		if(i!=EQU_NUM-1){
			for(int j = i*num ;j<(i+1)*num;j++){
				float ms = pow(input->points[j].x,2)+pow(input->points[j].y,2);
				std::cout<<"ms:"<<ms<<std::endl;
				vect.push_back(ms);
			}
		}else{
			for(int j = i*num ;j<input->points.size();j++){
				float ms = pow(input->points[j].x,2)+pow(input->points[j].y,2);
				vect.push_back(ms);
			}
		}

		auto it = std::min_element(std::begin(vect), std::end(vect));
		
		std::cout << "index of smallest element: " << *it<< std::endl;
		std::cout << "value: " << vect[std::distance(std::begin(vect), it)] << std::endl;
		
		outCloud.push_back(input[vect[std::distance(std::begin(vect), it)]]);
	}
	std::cout <<"size : " <<outCloud.points.size()<<std::endl;
	cloud_ptr=outCloud.makeShared();
}
*/
void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	sensor_msgs::PointCloud2 input_pointcloud;

    projector.projectLaser(*msg, input_pointcloud); // laserscan to pcl2
    pcl::PointCloud<pcl::PointXYZ> rawCloud;
    pcl::fromROSMsg(input_pointcloud, rawCloud);

	std::cout <<"size : " <<rawCloud.points.size()<<std::endl;
	
	//reduce_pcl();
	//std::cout <<"size : " <<rawCloud.points.size()<<std::endl;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "laser");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("scan", 100, scan_callback);
	ros::spin();
	return 0;
}
