#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

def talker():
    rospy.init_node('Path_Publisher', anonymous=True)
    time.sleep(0.25)
    pub = rospy.Publisher('path', Path, queue_size=10)

    fl = open("hallwayLoopMission.txt")
    wp_list = []
    for x in fl:
        tmp = x.split(",")
        tst = list([float(a) for a in tmp])
        wp_list.append(tst)

    [n,m] = np.shape(wp_list)
    path_msg = Path()
    for i in range(n):
        temp_pose = PoseStamped()
        temp_pose.pose.position.x = wp_list[i][0]
        temp_pose.pose.position.y = wp_list[i][1]
        path_msg.poses.append(temp_pose)

    rospy.sleep(0.5)
    pub.publish(path_msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
