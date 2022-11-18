#!/usr/bin/env python3
from os import times
import rospy
import math
import tf
import tf2_ros as tf2
import numpy as np
import ros_numpy
import geometry_msgs.msg
from gazebo_ros import gazebo_interface
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Twist, WrenchStamped, TransformStamped

import sys
import cv2
import os
import math

import moveit_commander
import moveit_msgs.msg
import tmc_control_msgs.msg
import trajectory_msgs.msg

class TF_MANAGER():
    def __init__(self):
        self._tfbuff = tf2.Buffer()
        self._lis = tf2.TransformListener(self._tfbuff)
        self._tf_static_broad = tf2.StaticTransformBroadcaster()
        self._broad = tf2.TransformBroadcaster()

    def _fillMsg(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = ref
        TS.child_frame_id = point_name
        TS.transform.translation.x = pos[0]
        TS.transform.translation.y = pos[1]
        TS.transform.translation.z = pos[2]
        TS.transform.rotation.x = rot[0]
        TS.transform.rotation.y = rot[1]
        TS.transform.rotation.z = rot[2]
        TS.transform.rotation.w = rot[3]
        return TS

    def pub_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name = '', new_frame = 'map'):
        try:
            traf = self._tfbuff.lookup_transform(new_frame, point_name, rospy.Time(0))
            translation, rotational = self.tf2_obj_2_arr(traf)
            self.pub_static_tf(pos = translation, rot = rotational, point_name = point_name, ref = new_frame)
            return True
        except:
            return False

    def getTF(self, target_frame='', ref_frame='map'):
        try:
            tf = self._tfbuff.lookup_transform(ref_frame, target_frame, rospy.Time(0))
            return self.tf2_obj_2_arr(tf)
        except:
            return [False,False]

    def tf2_obj_2_arr(self, transf):
        pos = []
        pos.append(transf.transform.translation.x)
        pos.append(transf.transform.translation.y)
        pos.append(transf.transform.translation.z)
    
        rot = []
        rot.append(transf.transform.rotation.x)
        rot.append(transf.transform.rotation.y)
        rot.append(transf.transform.rotation.z)
        rot.append(transf.transform.rotation.w)

        return [pos, rot]

def callback_base_pos(msg):
    global base_pos
    base_pos = np.asarray(msg.data)
        
def main():
	
	print('ETAPA 05 - UDEG')
	
	global base_pos
	    	
	rospy.init_node("etapa05")
	rospy.Subscriber("/base_link_coords", Float32MultiArray,callback_base_pos)
	pub_coords = rospy.Publisher("/meta_coords", Float32MultiArray, queue_size=10)
	loop = rospy.Rate(10)
	
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	
	# Esperar por los mensajes
	rospy.wait_for_message('/base_link_coords', Float32MultiArray, timeout=10000)
	
	# 1. Moverse a las coordenadas (1.3, 0.0)
	print('P1 Moviendo base a las coordenadas (1.3, 0.0)...')
	meta_pos = [1.3, 0.0, 0.0]
	
	meta_coords = Float32MultiArray()
	meta_coords.data = meta_pos
	
	dis_error = math.sqrt(math.pow(meta_pos[0]-base_pos[0],2)+math.pow(meta_pos[1] - base_pos[1], 2))
	angle_error = math.atan2(math.sin(meta_pos[2]-base_pos[2]), math.cos(meta_pos[2]-base_pos[2]))
	while dis_error >= 0.01 or abs(angle_error) >= 0.01:
		dis_error = math.sqrt(math.pow(meta_pos[0]-base_pos[0],2)+math.pow(meta_pos[1] - base_pos[1], 2))
		angle_error = math.atan2(math.sin(meta_pos[2]-base_pos[2]), math.cos(meta_pos[2]-base_pos[2]))
		pub_coords.publish(meta_coords)
		loop.sleep()
		
	print('P1 Base en las coordenadas: (', "{:.4f}".format(base_pos[0]), ', ', "{:.4f}".format(base_pos[1]), ').')
	
	# 2. Subir brazo robótico.
	print('P2 Subiendo brazo robótico...')
	
	arm = moveit_commander.MoveGroupCommander('arm')
	arm.set_num_planning_attempts(10)
	arm.set_planning_time(5)
	arm.set_named_target('go')
	arm.go(np.array((0.6, 0, 0.0, 0.0, 0, 0)))
	
	print('P2 Finalizado.')
	
	# 2. Bajar brazo robótico.
	print('P3 Bajando brazo robótico...')
	
	arm = moveit_commander.MoveGroupCommander('arm')
	arm.set_num_planning_attempts(10)
	arm.set_planning_time(5)
	arm.set_named_target('go')
	arm.go(np.array((0.0, 0, 0.0, 0.0, 0, 0)))
	
	print('P3 Listo.')
	
	# 3. Girar cabeza izquierda.
	print('P4 Girar cabeza izquierda...')
	
	head = moveit_commander.MoveGroupCommander('head')
	head.go(np.array((0.5*np.pi,0)))
	
	print('P5 Girar cabeza derecha...')
	head.go(np.array((0.0*np.pi,0)))
	
	print('P5 Listo.')
	
	

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
