#!/usr/bin/env python
#coding=utf-8
import rospy
import ros_numpy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg

EQU_NUM = 10

def reduce_pcl(before_pcl,pcl_number):
	after_pcl = np.zeros((10, 3))
	increment = int(pcl_number/EQU_NUM)
	for i in range(EQU_NUM):
		if i != EQU_NUM-1:
			idx = np.argmin(np.linalg.norm(before_pcl[i*increment:(i+1)*increment],axis=1,keepdims=True))
		else :
			idx = np.argmin(np.linalg.norm(before_pcl[i*increment:],axis=1,keepdims=True))
		after_pcl[i]=before_pcl[idx]
	return after_pcl

def scan_cb(data):	
	print(len(data.ranges))
	lp = lg.LaserProjection()
	pc2_data = lp.projectLaser(data)
	xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)

	#print(np.linalg.norm(xyz,axis=1,keepdims=True))
	pcl = xyz

	pcl_o3d = o3d.geometry.PointCloud()
	pcl_o3d.points = o3d.utility.Vector3dVector(pcl)
	downpcl = pcl_o3d.voxel_down_sample(voxel_size=0.1)
	pcl_o3d = np.asarray(downpcl.points)
	num = pcl_o3d.shape[0]
	print(num)
	pcl_o3d = reduce_pcl(pcl_o3d,num)
	print(pcl_o3d)
	pcl_o3d = pcl_o3d[np.abs(pcl_o3d[:, 0]) < 1.0]
	pcl_o3d = pcl_o3d[np.abs(pcl_o3d[:, 1]) < 1.0]
	print(pcl_o3d)
	
	
def listener():
    rospy.init_node('lidar_test', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_cb)
    rospy.spin()

if __name__ == '__main__':
    listener()
