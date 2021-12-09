#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped,Vector3Stamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import Quaternion
import math
#import getch
import sys
import keyboard
from std_msgs.msg import String #String message 
from std_msgs.msg import Int8
from ecbf_lidar.srv import *
import kbhit



class Controller:
    def __init__(self):
        self.current_odom=Odometry()
        self.current_state = State()
        self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)
        self.local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.local_acc_pub = rospy.Publisher("/mavros/setpoint_accel/accel", Vector3Stamped, queue_size=10)
        self.qp_client = rospy.ServiceProxy('qp', qp)
        self.rate=rospy.Rate(20)
        self.TwistStamped=TwistStamped()

    def quaternion_to_euler_angle(self,x, y, z,w):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z

    def body_to_world(self,vel_cmd):
        q_x=self.current_odom.pose.pose.orientation.x
        q_y=self.current_odom.pose.pose.orientation.y
        q_z=self.current_odom.pose.pose.orientation.z
        q_w=self.current_odom.pose.pose.orientation.w
        roll,pitch,yaw=self.quaternion_to_euler_angle(q_x, q_y, q_z,q_w)
        Rz=np.array([[math.cos(yaw)*math.cos(pitch),    math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll), math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll)],

                     [math.sin(yaw)*math.cos(pitch),    math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll), math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll)],

                     [-math.sin(pitch),                     math.cos(pitch)*math.sin(roll),         math.cos(pitch)*math.cos(roll)]

                    ])
        vel = np.array([vel_cmd[0],vel_cmd[1],vel_cmd[2]])
        wf_vel_cmd=np.dot(Rz,vel)



        return wf_vel_cmd[0], wf_vel_cmd[1], wf_vel_cmd[2], 0

    def commander(self, acc_cmd):
        # acc = [0,0,0,0]
        # acc[0],acc[1],acc[2],acc[3] = self.body_to_world(acc_cmd)
        ez=3-self.current_odom.pose.pose.position.z
        cmd = Vector3Stamped()
        cmd.vector.x = acc_cmd[0]
        cmd.vector.y = acc_cmd[1]
        cmd.vector.z = acc_cmd[2]
        
        #print("new acc:"+str(acc_cmd)+"\n")
        vel=TwistStamped()
        
        # vel.twist.linear.x = 0
        # vel.twist.linear.y = 0
        vel.twist.linear.z = 0
        vel.twist.linear.z = 0.8*ez
        self.local_vel_pub.publish(vel)
        self.local_acc_pub.publish(cmd)

    def odom_cb(self, data):
        self.current_odom=data

    def takeoff(self, x, y, z):
        Kp=1.5
        w=0
        while True:
            ex=x-self.current_odom.pose.pose.position.x
            ey=y-self.current_odom.pose.pose.position.y
            ez=z-self.current_odom.pose.pose.position.z

            vel=TwistStamped()
            vel.twist.linear.x = Kp*ex
            vel.twist.linear.y = Kp*ey
            vel.twist.linear.z = Kp*ez
            self.local_vel_pub.publish(vel)
            self.rate.sleep()
            if abs(ex)<=0.05 and abs(ey)<=0.05 and abs(ez)<=0.05 :
                vel.twist.linear.x = 0
                vel.twist.linear.y = 0
                vel.twist.linear.z = 0
                self.local_vel_pub.publish(vel)
                break

    def keys(self):
        rate = rospy.Rate(10)#try removing this line ans see what happens

        kb = kbhit.KBHit()
        acc = [0,0,0]
        now_acc = [0,0,0]
        while not rospy.is_shutdown():
            acc = now_acc
            if kb.kbhit(): #If a key is pressed:
                k = kb.getch() #Detect what key was pressed
                if k == 'A':
                    acc = [0.4,0,0]
                    rospy.loginfo("up") # to print on  terminal
                elif k =='B' :
                    acc = [-0.4,0,0]
                    rospy.loginfo("down") # to print on  terminal
                elif k == 'C' :
                    acc = [0,0.4,0]
                    rospy.loginfo("right") # to print on  terminal
                elif k == 'D':
                    acc = [0,-0.4,0]
                    rospy.loginfo("left") # to print on  terminal
                elif k == '0':
                    acc = [0,0,0]
                

                else:
                    print("nothing")

            print("origin:\n acc[0]:%.5f\n acc[1]:%.5f\n acc[2]:%.5f\n" %(acc[0],acc[1] ,acc[2]))
                
            try :
                
                res = self.qp_client(acc)
                now_acc = acc
                acc = res.ecbf_output
                print("after qp:\n acc[0]:%.5f\n acc[1]:%.5f\n acc[2]:%.5f\n" %(acc[0],acc[1] ,acc[2]))
            except:
                print("fail to call qp!")
            print("-----------")
            self.commander(acc)
            rate.sleep() 


if __name__ == "__main__":

    rospy.init_node("key_control")
    controller = Controller()
    controller.takeoff(0, 0, 3)
    print('finish')
    while not rospy.is_shutdown():
        controller.keys()

