#! /usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Quaternion, Pose
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
#from tf_conversions import quaternion_to_euler


class roverDriftControl:
    def __init__(self):

        self.max_str_angle = rospy.get_param('max_str_angle',0.75)
        self.max_thr = rospy.get_param('max_throttle',0.2)
        self.desSpeed = rospy.get_param('desired_speed',1)
        self.dist_threshold = rospy.get_param('waypoint_distance_threshold',0.4) # default to 10 cm
        self.L = rospy.get_param('wheel_base',0.2) # 0.2 meters
        self.KpSpd = rospy.get_param('Kp_spd',1.0)
        self.KiSpd = rospy.get_param('Ki_spd',0.05)
        self.KpYaw = rospy.get_param('Kp_yaw',3.5)

        self.acker_pub = rospy.Publisher('acker_cmd',AckermannDriveStamped,queue_size=5)
        self.wp_sub = rospy.Subscriber('waypoint',Pose,self.wptCallBack)
        self.path_sub = rospy.Subscriber('path',Path,self.pathCallBack)
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

    def wptCallBack(self,msg):
        self.xdes = msg.position.x
        self.ydes = msg.position.y
        self.mission = True

    def pathCallBack(self,msg):

        print("%d",len(msg.poses))
        self.wp_n = len(msg.poses)
        self.wp_list = np.zeros([2,self.wp_n])
        for i in range(self.wp_n):
            self.wp_list[0,i] = msg.poses[i].pose.position.x
            self.wp_list[1,i] = msg.poses[i].pose.position.y

        print(self.wp_list)
        self.mission = True

        #self.xdes = msg.position.x # Modify for path
        #self.ydes = msg.position.y

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




if __name__ == '__main__':
    rospy.init_node('waypoint_navigation')

    myRover = roverWaypointControl()

    rospy.spin()
