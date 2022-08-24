#!/usr/bin/env python3

# Nodo que publica la pose del base_link de Takeshi con respecto al marco de referencia 'map'.
# La pose se publica en el tópico /base_link_coords en un mensaje tipo Float32MultiArray
# con un formato [x, y, theta_rad].

import rospy
import tf
import tf2_ros
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
    
def PoseStamped_to_pose(msg):
    global meta_coords
    # PoseStamped_to_pose2d() convierte un mensaje tipo PoseStamped a una pose 2d en un arreglo
    # de Numpy con la forma (x,y,theta) con theta en radianes.
    
    x = msg.pose.position.x
    y = msg.pose.position.y
    (_, _, theta) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    meta_coords =  list([x, y, theta])

def get_base_link_pose():
    # get_base_link_pose() regresa un arreglo en Numpy con la pose de la base del robot con
    # respecto al marco de referencia global en la forma (x, y, theta) con theta en radianes.
    
    trans = get_base_link_coords()
    
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    (_, _, theta) = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

    return list([x, y, theta])

def get_base_link_coords():
    for i in range(10):
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            return trans
        except:
            print ('waiting for tf')
            trans = 0

if __name__ == '__main__':
    global meta_coords
    rospy.init_node("base_link_coords")
    rospy.Subscriber("/meta_competencia", PoseStamped,PoseStamped_to_pose)
    pub_coords = rospy.Publisher("/base_link_coords", Float32MultiArray, queue_size=10)
    pub_meta_coords = rospy.Publisher("/meta_coords", Float32MultiArray, queue_size=10)
    # Iniciar el árbol de transformaciones. Se necesita un delay para dar tiempo a cargar los
    # marcos de referencia.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1)
    
    loop = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        if meta_coords != False:
            base_coords = get_base_link_pose()
            msg_base_coords = Float32MultiArray()
            msg_meta_coords = Float32MultiArray()
            msg_base_coords.data = base_coords
            msg_meta_coords.data = meta_coords
            pub_coords.publish(msg_base_coords)
            pub_meta_coords.publish(msg_meta_coords)
            print("Meta: ",meta_coords)
            print("Base: ",base_coords)
        
        loop.sleep()

    
