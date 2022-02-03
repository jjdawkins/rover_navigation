#! /usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, Pose, Polygon, Point32
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
#from tf_conversions import quaternion_to_euler

# exectutes pose controller from atsushi code and Corker Robotics book


class ObstaclePublisher:
    def __init__(self):

        self.detection_threshold = rospy.get_param('detection_distance_threshold',1.5) # default to 1.5 m
        self.fov_threshold = rospy.get_param('FOV',math.pi/2)
        self.obstacle_pub = rospy.Publisher('obstacle',Polygon,queue_size=5)
        self.qtm_sub = rospy.Subscriber('odom',Odometry,self.odomCallBack)
        self.send_timer = rospy.Timer(rospy.Duration(0.1), self.sendCallBack)
        self.ob_list_g = np.array([[1,6],[0,1.5]])
        self.ob_list_b = np.zeros([2,3])
        self.dist = 0.0
        self.xpos = 0.0
        self.ypos = 3

    def odomCallBack(self,msg):
        orientation_q = msg.pose.pose.orientation # extract quaternion from pose message
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # put quaternion elements into an array
        euler = tf.transformations.euler_from_quaternion(orientation_list)
        self.yaw = euler[2]

        #self.speed = math.sqrt(speedx*speedx + speedy*speedy)
        self.xpos = msg.pose.pose.position.x # parse x-component of position from odometry message
        self.ypos = msg.pose.pose.position.y # parse y-component of position from odometry message

        # need to calculate relative positiion vector, then rotate
        #rospy.loginfo(self.xpos)
        for i in range(self.ob_list_g.shape[0]):
            xrel = self.ob_list_g[i,0]-self.xpos
            yrel = self.ob_list_g[i,1]-self.ypos
            self.ob_list_b[i,0] = math.cos(self.yaw)*xrel + math.sin(self.yaw)*yrel # x-position in body fixed frame
            self.ob_list_b[i,1] = -math.sin(self.yaw)*xrel + math.cos(self.yaw)*yrel # y-position in body fixed frame
            self.ob_list_b[i,2] = math.atan2(self.ob_list_b[i,1],self.ob_list_b[i,0]) # angle to obstacle center in body-fixed frame
            # also need to check if point is in FOV of car

    def sendCallBack(self,msg):
        tmp = Polygon()
        count = 0
        for i in range(self.ob_list_b.shape[0]):
            # calculate how far I am from waypoint
            dist = math.sqrt((self.ob_list_b[i,0])**2 + (self.ob_list_b[i,1])**2)
            #rospy.loginfo("%f,%f",i,dist)
            if (dist<=self.detection_threshold and abs(self.ob_list_b[i,2]) <= self.fov_threshold):
                tmp2 = Point32()
                # package polygon message
                tmp2.x = self.ob_list_b[i,0]
                tmp2.y = self.ob_list_b[i,1]
                tmp.points.append(tmp2)
                count += 1
        #rospy.loginfo(count)
        if count>0:
            # publish point message
            rospy.loginfo("Publishing %f obstacle(s)",count)
            self.obstacle_pub.publish(tmp)
        else:
            rospy.loginfo("Publishing 0 obstacle(s)")




if __name__ == '__main__':
    rospy.init_node('obstacle_publisher')

    Obstacles = ObstaclePublisher()

    rospy.spin()
