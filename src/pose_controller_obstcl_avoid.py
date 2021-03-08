#! /usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Quaternion, Pose, Polygon
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
#from tf_conversions import quaternion_to_euler

# exectutes pose controller from atsushi code and Corker Robotics book


class roverWaypointControlObst:
    def __init__(self):

        self.max_str_angle = rospy.get_param('max_str_angle',0.75)
        self.max_thr = rospy.get_param('max_throttle',0.2)
        self.desSpeed = rospy.get_param('desired_speed',0.3)
        self.dist_threshold = rospy.get_param('waypoint_distance_threshold',0.4) # default to 10 cm
        self.L = rospy.get_param('wheel_base',0.2) # 0.2 meters
        self.KpSpd = rospy.get_param('Kp_spd',1.0)
        self.KiSpd = rospy.get_param('Ki_spd',0.05)
        self.KpYaw = rospy.get_param('Kp_yaw',3.5)
        self.KpAvoid = rospy.get_param('Kp_avoidance',8)
        self.c3 = rospy.get_param('avoidanceC3',0.9) # 1.6119
        self.c4 = rospy.get_param('avoidanceC4',0.5) # 13.018
        self.c5 = rospy.get_param('avoidancec5',0.9) # 6.08
        self.dmax = rospy.get_param('ObstacleAvoidanceRadius',1)

        self.acker_pub = rospy.Publisher('acker_cmd',AckermannDriveStamped,queue_size=5)
        self.wp_sub = rospy.Subscriber('waypoint',Pose,self.wptCallBack)
        self.path_sub = rospy.Subscriber('path',Path,self.pathCallBack)
        self.obst_sub = rospy.Subscriber('obstacle',Polygon,self.obstacleCallback)
        self.qtm_sub = rospy.Subscriber('odom',Odometry,self.odomCallBack)
        self.send_timer = rospy.Timer(rospy.Duration(0.1), self.sendCallBack)

        self.ackMsg = AckermannDriveStamped()
        self.twistMsg = Twist()
        self.yawRate = 0.0 # u2 variable
        self.xdes = 2.0
        self.ydes = 2.0
        self.xpos = 0.0
        self.ypos = 0.0
        self.xerr = 0.0
        self.yerr = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.dist = 0.0
        self.spdErrInt = 0
        self.wp_list = []
        self.wp_n = 0
        self.wp_ind = 0
        self.yaw_des = 0
        self.xvel_b = 0
        self.yvel_b = 0
        self.xvel_g = 0
        self.yvel_g = 0
        self.mission = False
        self.my_path = Path()
        self.ob_n = 0
        self.ob_list = np.zeros([2,self.ob_n])

    def wptCallBack(self,msg):
        self.xdes = msg.position.x
        self.ydes = msg.position.y
        self.mission = True

    def pathCallBack(self,msg):

        print("%d",len(msg.poses))
        self.wp_n = len(msg.poses)
        self.wp_list = np.zeros([3,self.wp_n])
        for i in range(self.wp_n):
            self.wp_list[0,i] = msg.poses[i].pose.position.x
            self.wp_list[1,i] = msg.poses[i].pose.position.y
            orientation_q = msg.poses[i].pose.orientation # extract quaternion from pose message
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # put quaternion elements into an array
            euler = tf.transformations.euler_from_quaternion(orientation_list)
            rospy.loginfo(euler)
            self.wp_list[2,i] = euler[2]

        print(self.wp_list)
        self.mission = True

        #self.xdes = msg.position.x # Modify for path
        #self.ydes = msg.position.y

    def obstacleCallback(self,msg):

        #rospy.loginfo(len(msg.points))
        self.ob_n = len(msg.points)
        self.ob_list = np.zeros([2,self.ob_n])
        for i in range(self.ob_n):
            self.ob_list[0,i] = msg.points[i].x
            self.ob_list[1,i] = msg.points[i].y
        #print(self.ob_list)

    def odomCallBack(self,msg):
        orientation_q = msg.pose.pose.orientation # extract quaternion from pose message
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # put quaternion elements into an array
        euler = tf.transformations.euler_from_quaternion(orientation_list)
        self.yaw = euler[2]

        #(roll, pitch, yaw) = euler_from_quaternion (orientation_list) # convert quaternion to Euler angles
        self.xvel_g = msg.twist.twist.linear.x # parse x-component of inertial speed from odometry message
        self.yvel_g = msg.twist.twist.linear.y # parse y-component of inertial speed from odometry message

        self.xvel_b = math.cos(self.yaw)*self.xvel_g + math.sin(self.yaw)*self.yvel_g
        self.yvel_b = -math.sin(self.yaw)*self.xvel_g + math.cos(self.yaw)*self.yvel_g

        #self.speed = math.sqrt(speedx*speedx + speedy*speedy)
        self.xpos = msg.pose.pose.position.x # parse x-component of position from odometry message
        self.ypos = msg.pose.pose.position.y # parse y-component of position from odometry message

    def sendCallBack(self,msg):
        # Steering controller (using yaw rate speed kinematics)
        q = [0,0,0,1]
        euler = tf.transformations.euler_from_quaternion(q)
        if(self.mission):
            steerAng = Float64()

            if(self.wp_ind == self.wp_n): # If we reached the last waypoint start over
                self.wp_ind = 0

            self.xdes = self.wp_list[0,self.wp_ind]
            self.ydes = self.wp_list[1,self.wp_ind]
            goal_ang = self.wp_list[2,self.wp_ind]

            self.xerr = self.xdes-self.xpos
            self.yerr = self.ydes-self.ypos

            #self.dist = math.sqrt(x_err*x_err + y_err*y_err)
            # calculate how far I am from waypoint
            self.dist = math.sqrt((self.xerr)**2 + (self.yerr)**2)

            if(self.dist < self.dist_threshold):
                self.wp_ind = self.wp_ind + 1


            self.yaw_des = math.atan2(self.yerr,self.xerr)
            yaw_position = (self.yaw_des-self.yaw) # alpha term in other code

            if yaw_position>math.pi:
                yaw_position = yaw_position-2*math.pi
            elif yaw_position<-math.pi:
                yaw_position = yaw_position+2*math.pi

            yaw_goal_beta = goal_ang-self.yaw-yaw_position

            if yaw_goal_beta>math.pi:
                yaw_goal_beta = yaw_goal_beta-2*math.pi
            elif yaw_goal_beta<-math.pi:
                yaw_goal_beta = yaw_goal_beta+2*math.pi


            ob_term = 0
            if self.ob_n>0:
                for i in range(self.ob_n):
                    angOb = math.atan2(self.ob_list[1,i],self.ob_list[0,i])
                    distOb = math.sqrt(self.ob_list[1,i]**2 + self.ob_list[0,i]**2)
                    dgv = distOb*abs(math.sin(2*self.yaw-angOb-self.yaw_des))
                    ob_term = ob_term + self.KpAvoid*(self.yaw-angOb)*math.exp(-self.c3*distOb)*math.exp(-self.c4*abs(self.yaw-angOb))*(1+self.c5*(self.dmax-min([self.dmax, dgv]))**2)
                    rospy.loginfo("CONSIDERING OBSTACLE")
                    rospy.loginfo("%f,%f,%f,%f",angOb,distOb,ob_term,dgv)
                    self.ob_n = 0



            self.yawRate = self.KpYaw*yaw_position - 0.2*self.KpYaw*yaw_goal_beta + ob_term
            if abs(self.xvel_b)<0.1: # if speed less than 10 cm/s, use speed invariant turn angle
                steerAng = math.atan(self.yawRate*self.L/0.1)
            else: # if speed greater than 10 cm/s, use speed dependent steering angle
                steerAng = math.atan(self.yawRate*self.L/self.xvel_b)

            if abs(steerAng)>self.max_str_angle:
                steerAng = math.copysign(self.max_str_angle,steerAng)



            # if vehicle is within distance threshold, stop
            #if self.dist<self.dist_threshold:
             #  self.thr_cmd = 0.0 # stop if within threshold of waypoint


            # speed controller (Using PI controller on acceleration)
            self.spdErr = self.desSpeed - self.xvel_b

            # update speed error integral term
            self.spdErrInt = self.spdErrInt + self.spdErr*0.1 # 10 Hz sampling rate
            # saturate speed error integral term at 2
            #if self.spdErrInt > 2:
            #    self.spdErrInt = 2.0
            if(self.spdErrInt > 0.05):
                self.spdErrInt = 0.05
            elif (self.spdErrInt < -0.05):
                self.spdErrInt = -0.05
            else:
                self.spdErrInt = self.spdErrInt;

            # PWM duty cycle for throttle control
            self.thr_cmd = self.KpSpd*self.spdErr + self.KiSpd*self.spdErrInt # PI controller

            # Saturate speed command at 20% duty cycle
            if self.thr_cmd > self.max_thr:
                self.thr_cmd = self.max_thr

            if self.thr_cmd < -self.max_thr:
                self.thr_cmd = -self.max_thr


            # package ackermann message
            self.ackMsg.drive.steering_angle = steerAng
            self.ackMsg.drive.acceleration = self.thr_cmd

            rospy.loginfo("%f,%f,%f,%f",self.xdes,self.ydes,self.xpos,self.ypos)
            # publish ackermann message
            self.acker_pub.publish(self.ackMsg)
        else:
            rospy.loginfo("Waiting for Mission")


if __name__ == '__main__':
    rospy.init_node('waypoint_navigation_obstacle_avoid')

    myRover = roverWaypointControlObst()

    rospy.spin()
