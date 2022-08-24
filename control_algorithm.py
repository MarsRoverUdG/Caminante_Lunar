#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

def callback_obstacle_angles(msg):
    global obstacle_angles
    obstacle_angles = np.asarray(msg.data)

def callback_obstacle_distances(msg):
    global obstacle_distances
    obstacle_distances = np.asarray(msg.data)

def callback_base_pos(msg):
    global base_pos
    base_pos = np.asarray(msg.data)

def callback_meta_pos(msg):
    global meta_pos
    meta_pos = np.asarray(msg.data)


if __name__ == '__main__':
    global obstacle_angles
    global obstacle_distances
    global base_pos
    global meta_pos
    k_att = 1
    k_rep = 5
    kvx = 0.6
    kvy = 0.6
    kw = 0.3
    h = 0.2
    rospy.init_node("control_algorithm_node")
    rospy.Subscriber("/obstacle_detect_angles",Float32MultiArray,callback_obstacle_angles)
    rospy.Subscriber("/obstacle_detect_distances",Float32MultiArray,callback_obstacle_distances)
    rospy.Subscriber("/base_link_coords", Float32MultiArray,callback_base_pos)
    rospy.Subscriber("/meta_coords",Float32MultiArray,callback_meta_pos)
    pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
    loop = rospy.Rate(10)
    obstacle_angles = np.zeros((18))
    obstacle_distances = np.zeros((18))
    base_pos = np.zeros((3))
    meta_pos = np.zeros((3))
    while not rospy.is_shutdown():
        ## CONTROL ALGORITHM ##
        
        #Force vectors calculus
        F_att = k_att*(meta_pos[0:2] - base_pos[0:2])
        F_rep_x = k_rep*(obstacle_distances - 1.5)*np.cos(obstacle_angles + base_pos[2])
        F_rep_y = k_rep*(obstacle_distances - 1.5)*np.sin(obstacle_angles + base_pos[2])
        F_x = F_att[0] + np.sum(F_rep_x)
        F_y = F_att[1] + np.sum(F_rep_y)
        #Force normalization
        distance_left = math.sqrt(math.pow(meta_pos[0]-base_pos[0],2) + pow(meta_pos[0]-base_pos[0],2))
        normF_x = F_x if distance_left <=0.3 else F_x/math.sqrt(math.pow(F_x,2) + math.pow(F_y,2))
        normF_y = F_y if distance_left <=0.3 else F_y/math.sqrt(math.pow(F_x,2) + math.pow(F_y,2))
        # Gradient step
        x_d = base_pos[0] + h*normF_x
        y_d = base_pos[1] + h*normF_y
        theta_d = math.atan2(y_d - base_pos[1],x_d - base_pos[0])
        #theta_d = meta_pos[2]
        if math.sqrt(math.pow(meta_pos[0] - x_d,2) + math.pow(meta_pos[1] - y_d,2)) <=0.1:
            x_d = meta_pos[0]
            y_d = meta_pos[1]
            theta_d = meta_pos[2]
        error_w = math.atan2(math.sin(theta_d-base_pos[2]),math.cos(theta_d-base_pos[2]))
        # Message sending
        msg_cmd_vel = Twist()
        if math.sqrt(math.pow(meta_pos[0] - base_pos[0],2) + math.pow(meta_pos[1] - base_pos[1],2)) >= 0.01:
            msg_cmd_vel.linear.x = kvx*(x_d - base_pos[0])
            msg_cmd_vel.linear.y = kvy*(y_d - base_pos[1])
            msg_cmd_vel.angular.z = kw*error_w
        else:
            msg_cmd_vel.linear.x = 0
            msg_cmd_vel.linear.y = 0
            msg_cmd_vel.angular.z = 0
        print("error x: ",x_d - base_pos[0])
        print("error y: ",y_d - base_pos[1])
        pub_cmd_vel.publish(msg_cmd_vel)        
        
        loop.sleep()

    