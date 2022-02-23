#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
import tf
import numpy as np


def wrapToPi(ang):
    while(ang >= math.pi):
        ang = ang - 2*math.pi

    while(ang <= -math.pi):
        ang = ang + 2*math.pi

    return ang

class controller:

    def __init__(self):
        self.Kp = rospy.get_param('~gains/kp',1.0)
        self.Kd = 0
        self.set_spd = rospy.get_param('~speed', 1.0)
        self.max_str_ang = rospy.get_param('~max_str_ang',0.25)
        self.wp_rad = rospy.get_param('~wp_radius',0.3)
        self.str_ang = 0
        self.tel_spd = 0
        self.tel_str = 0
        self.max_spd = rospy.get_param('~max_spd',1.5)
        self.yaw = 0
        self.des_yaw = 0
        self.xpos = 0
        self.ypos = 0
        self.xdes = 0
        self.ydes = 0
        self.wp_ind = 0

        self.acker_pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=10)

        self.path_sub = rospy.Subscriber("path",Path,self.pathCallBack)
        #self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallBack)
        self.odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amclCallBack)
        self.joy_sub = rospy.Subscriber("/joy",Joy,self.joyCallBack)
        self.init_time = rospy.get_time()
        self.wp_list = []
        self.path_received = False
        self.auto =  False
        #self.wp_list = np.array([[2, 1], [2, 4], [-3, 4],[-3, 1]])


        # Controller Starts at the end of the init
        self.timer = rospy.Timer(rospy.Duration(0.05), self.controlLoop)

    def joyCallBack(self,msg):
        #self.str_ang = self.max_str_ang*msg.axes[0]
        self.tel_spd = self.max_spd*msg.axes[4]
        self.tel_str = self.max_str_ang*msg.axes[1]

        if(msg.buttons[0]):
            self.auto = False

        if(msg.buttons[1]):
            self.auto = True

    def pathCallBack(self,msg):
        print(len(msg.poses))
        self.wp_list = np.zeros([len(msg.poses),2])
        for i in range(len(msg.poses)):
             self.wp_list[i,0] = msg.poses[i].pose.position.x
             self.wp_list[i,1] = msg.poses[i].pose.position.y

        self.path_received = True

    def odomCallBack(self,msg):
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        #print(self.yaw)

    def amclCallBack(self,msg):
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        #print(self.yaw)

    def controlLoop(self,event):

        t = rospy.get_time()-self.init_time
        ack_msg = AckermannDriveStamped()
        if(self.path_received):
            #if(t > 5):
            #    self.des_yaw += 1.57
            #    self.init_time = rospy.get_time()
            self.xdes = self.wp_list[self.wp_ind,0]
            self.ydes = self.wp_list[self.wp_ind,1]

            xerr = self.xdes - self.xpos
            yerr = self.ydes - self.ypos

            self.des_yaw = math.atan2(yerr,xerr)
            self.dist = math.sqrt(xerr*xerr + yerr*yerr)

            if(self.dist < abs(self.wp_rad)):
                print("waypoint",self.wp_ind+1 ,"achieved")
                self.wp_ind = self.wp_ind + 1
                [n,m] = np.shape(self.wp_list)
                if(self.wp_ind == n):
                    self.wp_ind = 0
                    self.path_received = False
            self.str_ang = np.sign(self.set_spd)*self.Kp*wrapToPi(self.des_yaw - self.yaw)

        else:
            print("No Path Received yet")

        if(self.auto):
            ack_msg.drive.steering_angle = self.str_ang
            ack_msg.drive.speed = self.set_spd
        else:
            ack_msg.drive.steering_angle = self.tel_str
            ack_msg.drive.speed = self.tel_spd
            #print(self.tel_str)

        self.acker_pub.publish(ack_msg)

if __name__ == '__main__':

    try:
        rospy.init_node('Controller_Node', anonymous=True)
        my_ctrl = controller()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
