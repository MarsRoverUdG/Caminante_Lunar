#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Twist

#--------Callback Functions-------------
def callback_time(msg):
    global reloj
    reloj = msg.data

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

#----------------Main--------------------
if __name__ == '__main__':
    global obstacle_angles
    global obstacle_distances
    global base_pos
    global meta_pos
    global reloj
    ## Control Parameters
    k_att = 1
    k_rep = 3
    kvx = 1
    kvy = 1
    kw = 0.6
    h = 0.15
    d_max = 0.8
    # Ros Node Config
    rospy.init_node("control_algorithm_node")
    rospy.Subscriber("/time",Int32,callback_time)
    rospy.Subscriber("/obstacle_detect_angles",Float32MultiArray,callback_obstacle_angles)
    rospy.Subscriber("/obstacle_detect_distances",Float32MultiArray,callback_obstacle_distances)
    rospy.Subscriber("/base_link_coords", Float32MultiArray,callback_base_pos)
    rospy.Subscriber("/meta_coords",Float32MultiArray,callback_meta_pos)
    pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
    loop = rospy.Rate(10)
    # Initial state
    n_sections = 24
    obstacle_angles = np.zeros((n_sections))
    obstacle_distances = np.zeros((n_sections))
    base_pos = np.asarray([2,2,2])
    meta_pos = np.zeros((3))
    reloj = 0
    inicio = 0
    final = 0
    while not rospy.is_shutdown():
        if reloj != 0 and inicio == 0:
            inicio = reloj
            print("Inicio: ",inicio)
        ## CONTROL ALGORITHM ##
        #Force vectors calculus
        F_att = k_att*(meta_pos[0:2] - base_pos[0:2])
        F_rep_x = k_rep*(obstacle_distances - d_max)*np.cos(obstacle_angles + base_pos[2])
        F_rep_y = k_rep*(obstacle_distances - d_max)*np.sin(obstacle_angles + base_pos[2])

        F_x = F_att[0] + np.sum(F_rep_x)
        F_y = F_att[1] + np.sum(F_rep_y)
        #Force normalization
        distance_left = math.sqrt(math.pow(meta_pos[0]-base_pos[0],2) + pow(meta_pos[1]-base_pos[1],2))
        normF_x = F_x if distance_left <=1.0 else F_x/math.sqrt(math.pow(F_x,2) + math.pow(F_y,2))
        normF_y = F_y if distance_left <=1.0 else F_y/math.sqrt(math.pow(F_x,2) + math.pow(F_y,2))
        # Gradient step
        x_d = base_pos[0] + h*normF_x 
        y_d = base_pos[1] + h*normF_y 
        #theta_d = math.atan2(y_d - base_pos[1],x_d - base_pos[0])
        theta_d = math.atan2(meta_pos[1] - base_pos[1],meta_pos[0] - base_pos[0])
        # If the step is close enough to the goal, then the reference is the goal
        if math.sqrt(math.pow(meta_pos[0] - x_d,2) + math.pow(meta_pos[1] - y_d,2)) <=0.6:
            x_d = meta_pos[0]
            y_d = meta_pos[1]
            theta_d = meta_pos[2]
        print("Reference: ",x_d,", ",y_d,", ",theta_d)
        # Errors calculations
        error_x = x_d - base_pos[0]
        error_y = y_d - base_pos[1]
        error_w = math.atan2(math.sin(theta_d-base_pos[2]),math.cos(theta_d-base_pos[2]))
        # Message sending
        msg_cmd_vel = Twist()
        # Distance between the robot base and the goal
        dis_error = math.sqrt(math.pow(meta_pos[0] - base_pos[0],2) + math.pow(meta_pos[1] - base_pos[1],2))
        # Condition to continue or stop the control proccess
        if dis_error >= 0.01 or error_w >= 0.01:
            msg_cmd_vel.linear.x = kvx*(error_x*math.cos(base_pos[2]) + error_y*math.sin(base_pos[2]))
            msg_cmd_vel.linear.y = kvy*(-error_x*math.sin(base_pos[2]) + error_y*math.cos(base_pos[2]))
            msg_cmd_vel.angular.z = kw*error_w
        else:
            msg_cmd_vel.linear.x = 0
            msg_cmd_vel.linear.y = 0
            msg_cmd_vel.angular.z = 0
            final = reloj
            print("Inicio: ",inicio," segundos")
            print("Final : ",final," segundos")
            print("Duración: ",final - inicio," segundos")
            break
        pub_cmd_vel.publish(msg_cmd_vel)        
        
        loop.sleep()