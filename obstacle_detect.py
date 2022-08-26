#!/usr/bin/env python3

# Detects the obstacles and select only 18 obstacle points

from pickletools import uint8
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import math

def callback_laser(msg):
    global distances    # Obstacle distances
    global angles       # Obstacle angles (respect the robot ref frame)
    # There are 721 laser data, from -120 to 120 degrees
    # We need only from -90 to 90 degrees, so we don care about the first and the last 90 data
    laser_sec = np.zeros((30,2))    # Section array (distance and index) 30 elements per section
    min_sec = np.zeros((18,2))      # Section minimum (distance and index) 18 sections
    d_max = 1.0
    ### Object Dectection algorithm ###
    for i in range(0,18):   # Number of dectections (18)
        laser_sec[:,0] = np.asarray([msg.ranges[i*30 + 90:i*30 + 120]])         # Get the distances of the section data
        laser_sec[:,1] = np.asarray([j for j in range(i*30 + 90,i*30 + 120)])   # Get the index of the section data
        #min_sec[i,0] = np.amin(laser_sec[:,0]) if np.amin(laser_sec[:,0]) <= 1.5 else 0.0                       # Get the minimum distance of the section data
        min_sec[i,0] = laser_sec[0,0] if laser_sec[0,0] <= d_max else d_max
        #min_sec[i,1] = laser_sec[np.argmin(laser_sec[:,0]),1]                   # Get the index of the minimum
        min_sec[i,1] = laser_sec[0,1]
    min_sec[:,1] = min_sec[:,1]*msg.angle_increment -120*math.pi/180            # Transform the idex to angle (rad) respect the robot ref frame

    #print(min_sec)
    #print("==========================")
    #laser = np.asarray(msg.ranges[90:631])
    #laser_sec = np.zeros((18, 30))
    #min_sec = np.zeros((18,3))
    #for i in range(18):
    #    laser_sec[i, :] = laser[30*i:30*(i)+30]
    #    min_sec[i,1] = np.amin(laser_sec[i, :])
    #    min_sec[i,2] = (90+np.argmin(laser_sec[i, :]) + 30*i) * msg.angle_increment
    #print(min_sec)
    #print("==========================")
    distances = min_sec[:,0].tolist()
    angles = min_sec[:,1].tolist()

def main():
    # Node config
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
            #print("Angles and distances are publishing")
            print("distances: ",distances)
            print("angles: ",angles)
        loop.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass