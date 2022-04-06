#!/usr/bin/env python
#coding=utf-8
import rospy
import ros_numpy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg

from cvxopt import matrix, solvers
import time
from rospy.rostime import Time
from geometry_msgs.msg import PoseStamped
from ecbf_lidar.srv import *

DEL_V = 0.3
upper_v = 4
lower_v = -4
EQU_NUM = 10

class ConstraintGenerator:
    def __init__(self):
        rospy.init_node("osqp_solver")
        self.pcl_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.pos_sub = rospy.Subscriber("/vrpn_client_node/MAV1/pose", PoseStamped, self.pos_cb)
        self.qp_server = rospy.Service('qp', qp, self.qp_handler)
        self.rate = rospy.Rate(100)
        self.pcl = None
		self.pcl_flag = False
        self.lp = lg.LaserProjection()
        self.safe_dis = 1.0

        self.pos = None
        self.vel = None
        self.pose_init_flag = False
        self.new_u = []

        solvers.options['show_progress'] = False
        self.P = matrix(np.identity(3))
        self.Q = matrix(np.zeros(3))
        self.G = None
        self.H = None
        self.k1 = 2
        self.k2 = 2

        self.pos = [0.0,0.0,0.0]
        self.last_pos = [0.0,0.0,0.0]
        self.vel = [0.0,0.0,0.0]
        self.last_vel = [0.0,0.0,0.0]
        self.last_time = 0.0
        self.now_time = 0.0

        self._initialize()
	
    def scan_cb(self, data):
        pc2_data = self.lp.projectLaser(data)
        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)
        #print(np.linalg.norm(xyz,axis=1,keepdims=True))
        self.pcl = xyz
		self.pcl_flag = True

	def reduce_pcl(self,before_pcl): # reduce pcl to appointed equation number 
		after_pcl = np.zeros((10, 3))
		pcl_number = before_pcl.shape[0]
		increment = int(pcl_number/EQU_NUM)
		for i in range(EQU_NUM):
			if i != EQU_NUM-1:
				idx = np.argmin(np.linalg.norm(before_pcl[i*increment:(i+1)*increment],axis=1,keepdims=True))
			else :
				idx = np.argmin(np.linalg.norm(before_pcl[i*increment:],axis=1,keepdims=True))
			after_pcl[i]=before_pcl[idx]
		return after_pcl

    def contraint_solver(self, u):
		if self.pcl_flag != True :
			print("do not get lidar data")
			return np.array([u[0], u[1], u[2]])
		else:
			pcl = o3d.geometry.PointCloud()
			pcl.points = o3d.utility.Vector3dVector(self.pcl)
			
			downpcl = pcl.voxel_down_sample(voxel_size=2)
			pcl = np.asarray(downpcl.points)
			pcl = reduce_pcl(pcl)
			pcl = pcl[np.abs(pcl[:, 0]) < 1.0]
			pcl = pcl[np.abs(pcl[:, 1]) < 1.0]
			pcl = -pcl
			#print(pcl)
			dis_sum_square=np.square(pcl).sum(axis=1)
			vel_sum_square=np.square(self.vel).sum(axis=0)

			g = -2*pcl
			h = 2*vel_sum_square*np.ones(len(pcl)) +\
				self.k1*(dis_sum_square-(self.safe_dis*self.safe_dis)*np.ones(len(pcl)))+\
				self.k2*2*(pcl.dot(self.vel))

			#print(g.shape)
			#print(h.shape)

			self.Q = matrix(-0.5*u[0:3],tc='d')
			self.G = matrix(g,tc='d')
			self.H = matrix(h,tc='d')
			solvers.options['feastol']=1e-4
			solvers.options['maxiters']=80
			sol=solvers.coneqp(self.P, self.Q, self.G, self.H)
			u_star = sol['x']
			#print(sol['s'])
			return np.array([u_star[0], u_star[1], u[2]]) # do not constraint z domain 

    def pos_cb(self, data):
        if self.pose_init_flag == False:
            self.now_time = data.header.stamp.to_sec()
            self.pos[0] = data.pose.position.x
            self.pos[1] = data.pose.position.y
            self.pos[2] = data.pose.position.z
            self.last_pos[0] = self.pos[0]
            self.last_pos[1] = self.pos[1]
            self.last_pos[2] = self.pos[2]
            self.last_time = self.now_time
            self.pose_init_flag = True

        else :
            self.now_time = data.header.stamp.to_sec()
            delta_time = self.now_time - self.last_time

            self.pos[0] = data.pose.position.x
            self.pos[1] = data.pose.position.y
            self.pos[2] = data.pose.position.z

            self.vel[0] =(self.pos[0] - self.last_pos[0])/0.0083
            self.vel[1] =(self.pos[1] - self.last_pos[1])/0.0083
            self.vel[2] =(self.pos[2] - self.last_pos[2])/0.0083

            self.vel[0] = self.fix_vel(self.vel[0],self.last_vel[0])
            self.vel[1] = self.fix_vel(self.vel[1],self.last_vel[1])
            self.vel[2] = self.fix_vel(self.vel[2],self.last_vel[2])

            self.last_vel[0] = self.vel[0]
            self.last_vel[1] = self.vel[1]
            self.last_vel[2] = self.vel[2]
            self.last_pos[0] = self.pos[0]
            self.last_pos[1] = self.pos[1]
            self.last_pos[2] = self.pos[2]
            self.last_time = self.now_time
            
        return 0

    def fix_vel(self,now_vel,old_vel):
        new_vel = old_vel
        delta_vel = 0.0
        delta_vel = now_vel-old_vel
        if (abs(delta_vel)>DEL_V) :
            if (delta_vel>0):
                new_vel +=DEL_V
            else:
                new_vel -=DEL_V  
        else:
            new_vel = now_vel
        new_vel = self.bound(new_vel)
        return new_vel

    def bound(self, v):
        if v>upper_v :
            v = upper_v
        elif v<lower_v:
            v = lower_v
        
        return v

    def qp_handler(self,data):
        print("call service")
        u =np.array([0.0,0.0,0.0])
        for i in range(3):
            u[i] = data.desire_input[i] 
        new_u = self.contraint_solver(u)
        print("vel:\n vel[0]:%.5f\t vel[1]:%.5f\t vel[2]:%.5f\t" %(self.vel[0],self.vel[1] ,self.vel[2]))
        print("u:\n u[0]:%.5f\t u[1]:%.5f\t u[2]:%.5f\t" %(u[0],u[1] ,u[2]))
        print("new_u:\n new_u[0]:%.5f\t new_u[1]:%.5f\t new_u[2]:%.5f\t" %(new_u[0],new_u[1] ,new_u[2]))
        return [new_u]

    def process(self):
        print("Ready to solve qp.")
	self.rate.sleep()
        rospy.spin()


    def _initialize(self):
        print("initial start")
        while self.pcl_flag != True:
            self.rate.sleep()
        print("initial done")

if __name__ == "__main__":
    cg = ConstraintGenerator()
    cg.process()
