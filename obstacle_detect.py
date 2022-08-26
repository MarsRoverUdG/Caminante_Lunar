#!/usr/bin/env python3

# Detects the obstacles and select only 18 obstacle points

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import math

def callback_laser(msg):
    global distances    # Obstacle distances
    global angles       # Obstacle angles (respect the robot ref frame)
    n_elements = 30
    #print("Angle_section: ",n_elements*msg.angle_increment*180/math.pi)
    n_sections = 24
    # There are 721 laser data, from -120 to 120 degrees
    #laser_sec = np.zeros((n_elements,2))    # Section array (distance and index) 30 elements per section
    min_sec = np.zeros((n_sections,2))      # Section data (distance and index) 24 sections
    d_max = 0.8
    ### Object Dectection algorithm ###
    for i in range(0,n_sections):   # Number of dectections (18)
        #laser_sec[:,0] = np.asarray([msg.ranges[i*n_elements:i*n_elements + n_elements]])         # Get the distances of the section data
        #laser_sec[:,1] = np.asarray([j for j in range(i*n_elements,i*n_elements + n_elements)])   # Get the index of the section data
        #min_sec[i,0] = np.amin(laser_sec[:,0]) if np.amin(laser_sec[:,0]) <= 1.5 else 0.0         # Get the minimum distance of the section data
        #min_sec[i,1] = laser_sec[np.argmin(laser_sec[:,0]),1]                                     # Get the index of the minimum
        #min_sec[i,0] = laser_sec[0,0] if laser_sec[0,0] <= d_max else d_max
        min_sec[i,0] = msg.ranges[i*n_elements] if msg.ranges[i*n_elements] <= d_max else d_max
        min_sec[i,1] = i*n_elements
    min_sec[:,1] = min_sec[:,1]*msg.angle_increment -120*math.pi/180            # Transform the idex to angle (rad) respect the robot ref frame

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