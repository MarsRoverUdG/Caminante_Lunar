#!/usr/bin/env python3

from pickletools import uint8
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan


def callback_laser(msg):
    global distances
    global angles
    laser = np.asarray(msg.ranges[90:631])

    laser_sec = np.zeros((18, 30))
    min_sec = np.zeros((18,3))
    for i in range(18):
        laser_sec[i, :] = laser[30*i:30*(i)+30]
        min_sec[i,1] = np.amin(laser_sec[i, :])
        min_sec[i,2] = (90+np.argmin(laser_sec[i, :]) + 30*i) * msg.angle_increment
    distances = min_sec[:,1].tolist()
    angles = min_sec[:,2].tolist()

def main():
    global distances
    global angles
    rospy.init_node("obstacle_detect_node")
    rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_laser)
    pub_angles = rospy.Publisher("/obstacle_detect_angles",Float32MultiArray, queue_size=10)
    pub_distances = rospy.Publisher("/obstacle_detect_distances",Float32MultiArray, queue_size=10)

    loop = rospy.Rate(10)
    angles = False
    distances = False
    while not rospy.is_shutdown():
        if angles != False:
            #print(angles)
            msg_angles = Float32MultiArray()
            msg_angles.data = angles
            pub_angles.publish(msg_angles)

            msg_distances = Float32MultiArray()
            msg_distances.data = distances
            pub_distances.publish(msg_distances)
            print("Angles and distances are publishing")
        loop.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass