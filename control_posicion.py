#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped

def callback_LaserScan(msg):
    print(len(msg.ranges))

def callback_meta(msg):
    print('Pose meta:  ', PoseStamped_to_pose(msg))
    print('Pose robot: ', get_base_link_pose())
    
    return

def get_base_link_coords():
    for i in range(10):
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            return trans
        except:
            print ('waiting for tf')
            trans = 0
    
def get_base_link_pose():
    # get_base_link_pose() regresa un arreglo en Numpy con la pose de la base del robot con
    # respecto al marco de referencia global en la forma (x, y, theta) con theta en radianes.
    
    trans = get_base_link_coords()
    
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    (_, _, theta) = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

    return np.array([x, y, theta])
    
def PoseStamped_to_pose(msg):
    # PoseStamped_to_pose2d() convierte un mensaje tipo PoseStamped a una pose 2d en un arreglo
    # de Numpy con la forma (x,y,theta) con theta en radianes.
    
    x = msg.pose.position.x
    y = msg.pose.position.y
    (_, _, theta) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    return np.array([x, y, theta])



if __name__ == '__main__':
    rospy.init_node("control_posicion")
    rospy.Subscriber("/meta_competencia", PoseStamped, callback_meta)
    rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_LaserScan)
    pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
    
    # Iniciar el árbol de transformaciones. Se necesita un delay para dar tiempo a cargar los
    # marcos de referencia.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1)
    
    loop = rospy.Rate(10)
    laser = Laser()
    
    while not rospy.is_shutdown():

        #msg_cmd_vel = Twist()
        #msg_cmd_vel.linear.x = 0.1
        #pub_cmd_vel.publish(msg_cmd_vel)        
        
        loop.sleep()

    
