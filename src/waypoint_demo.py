#! /usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Quaternion, Pose, PoseStamped
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
#from tf_conversions import quaternion_to_euler


class waypointDemo:
    def __init__(self):

        self.max_str_angle = rospy.get_param('max_str_angle',0.6)
        self.max_speed = rospy.get_param('max_speed',2)
        self.desSpeed = rospy.get_param('desired_speed',0.4)
        self.dist_threshold = rospy.get_param('waypoint_distance_threshold',0.4) # default to 10 cm

        self.wp_pub = rospy.Publisher('waypoint',Pose,queue_size=5)
        self.path_pub = rospy.Publisher('path',Path,queue_size=5)




    def sendPath(self):

        my_path = Path()
        points = np.zeros((2,5))
        points[0,:] = (2,4,4,-2,-3)
        points[1,:] = (2,2,5,2,5)


        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        my_path.header = h
        (m,n) = np.shape(points) # dimensions of the matrix (x,y,z by number of points)
    # Convert points of specified polygon into desired lat loong and alt
        for i in range(n): # iterate over points in polygon
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = points[0,i]
            tmp_pose.pose.position.y = points[1,i]
            rospy.loginfo("%f,%f",points[0,i],points[1,i])
            my_path.poses.append(tmp_pose) # append points to polygon object
            #self.xdes = msg.position.x # Modify for path
            #self.ydes = msg.position.y
        print(my_path)
        self.path_pub.publish(my_path)

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



if __name__ == '__main__':
    rospy.init_node('waypoint_demo')

    myRover = waypointDemo()
    rospy.sleep(5)
    myRover.sendPath()

    rospy.spin()
