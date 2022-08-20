#!/usr/bin/env python3

from pickletools import uint8
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan

def callback_laser(msg):
    laser = np.asarray(msg.ranges[90:631])

    laser_sec = np.zeros((18, 30))
    min_sec = np.zeros((18,3))
    for i in range(18):
        laser_sec[i, :] = laser[30*i:30*(i)+30]
        min_sec[i,1] = np.amin(laser_sec[i, :])
        min_sec[i,0] = np.argmin(laser_sec[i, :]) + 30*i
        min_sec[i,2] = (90+min_sec[i,0]) * msg.angle_increment
    print("=============================")

    print(min_sec)

if __name__ == '__main__':

    rospy.init_node("obstacle_detect_node")
    rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_laser)
    

    loop = rospy.Rate(10)

    while not rospy.is_shutdown():
        loop.sleep()