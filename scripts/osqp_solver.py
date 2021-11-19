
import rospy
import ros_numpy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg
from controller import Controller
from cvxopt import matrix, solvers
import time
from geometry_msgs import PoseStamped

"""
implement constraint generator

TODO:
    0. Knowing how to use CBF (paper reading)
    1. (Maybe) need coordinate transform
    2. Design P, Q, G, H. Refer to cvxopt tutorial
    3. Instead of publishing command randomly, implement a keyboard commander (optional)
"""

class ConstraintGenerator:
    def __init__(self):
        self.pcl_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.pos_sub = rospy.Subscriber("/vrpn_client_node/MAV1/pose", geometry_msgs.PoseStamped, self.pos_cb)
        self.rate = rospy.Rate(20)
        self.pcl = None
        self.lp = lg.LaserProjection()
        self.safe_dis = 1.0

        self.pos = None
        self.vel = None
        self.pose_init_flag = False


        solvers.options['show_progress'] = False
        self.P = matrix(np.identity(3))
        self.Q = matrix(np.zeros(3))
        self.G = None
        self.H = None

        self.pos = [0.0,0.0,0.0]
        self.vel = [0.0,0.0,0.0]
        self.last_vel = [0.0,0.0,0.0]

        self._initialize()

    def scan_cb(self, data):
        pc2_data = self.lp.projectLaser(data)
        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)
        #print(np.linalg.norm(xyz,axis=1,keepdims=True))
        self.pcl = xyz

    def contraint_solver(self, u):
        pcl = o3d.geometry.PointCloud()
        pcl.points = o3d.utility.Vector3dVector(self.pcl)
        downpcl = pcl.voxel_down_sample(voxel_size=0.1)
        pcl = np.asarray(downpcl.points)
        pcl = pcl[np.abs(pcl[:, 0]) < 10.0]
        pcl = pcl[np.abs(pcl[:, 1]) < 10.0]
        pcl = pcl[np.abs(pcl[:, 2]) < 0.5]
        print(np.linalg.norm(pcl,axis=1,keepdims=True))
        row_sum_square=np.square(pcl).sum(axis=1)
        h=row_sum_square-self.safe_dis*self.safe_dis
        self.Q = matrix(-1*u[0:3],tc='d')
        self.G = matrix(pcl,tc='d')
        self.H = matrix(h,tc='d')
        #solvers.options['feastol']=1e-5
        sol=solvers.coneqp(self.P, self.Q, self.G, self.H)
        u_star = sol['x']
        #print(u_star)
        #print(sol['s'])

        return np.array([u_star[0], u_star[1], u_star[2],u[3]])

    def pos_cb(self, data):
		
		if pose_init_flag == false :
			now_time = data.header.stamp.toSec()
			pos[0] = data.pose.position.x
			pos[1] = data.pose.position.y
			pos[2] = data.pose.position.z
			last_pos[0] = pos[0]
			last_pos[1] = pos[1]
			last_pos[2] = pos[2]
			last_time = now_time
			pose_init_flag = true

		else :
			now_time = data.header.stamp.toSec()
            delta_time = now_time - last_time

			pos[0] = data.pose.position.x
			pos[1] = data.pose.position.y
			pos[2] = data.pose.position.z

			vel[0] =(pos[0] - last_pos[0])/0.0083
			vel[1] =(pos[1] - last_pos[1])/0.0083
			vel[2] =(pos[2] - last_pos[2])/0.0083

			vel[0] = fix_vel(vel[0],last_vel[0])
			vel[1] = fix_vel(vel[1],last_vel[1])
			vel[2] = fix_vel(vel[2],last_vel[2])

			last_vel[0] = vel[0]
			last_vel[1] = vel[1]
			last_vel[2] = vel[2]
			last_pos[0] = pos[0]
			last_pos[1] = pos[1]
			last_pos[2] = pos[2]
			last_time = now_time

    def fix_vel(now_vel,old_vel):
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

        new_vel = bound(new_vel)
        return new_vel
	
}


    def _initialize(self):
        while self.pcl is None:
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("osqp_solver")
    controller = Controller()
    cg = ConstraintGenerator()

    controller.takeoff(0, 0, 1.5)
    while not rospy.is_shutdown():
        desired_vel_cmd = np.array([0.5,0.5,0,0])
        start = time.time()
        constrained_vel_cmd = cg.contraint_solver(desired_vel_cmd)
        end = time.time()
        print (end-start)
        controller.commander(constrained_vel_cmd)