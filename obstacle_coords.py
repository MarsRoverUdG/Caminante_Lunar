#!/usr/bin/env python3

# Nodo que publica las coordenadas globales de los obstáculos visibles para el scan laser en  
# un arco de 180°. Se publican 18 obstáculos representativos del mapa utilizando información 
# de la posición de Takeshi, la posición de su laser_scan y los objetos detectados por este.
# La información publicada son las coordenadas globales de los obstáculos en los tópicos
# /obstacle_coords_x y /obstacle_coords_y en formato Float32MultiArray.

import rospy
import tf
import tf2_ros
import numpy as np
from std_msgs.msg import Float32MultiArray

def callback_angles(msg):
    global obstacle_angles
    obstacle_angles = msg.data
    
    return
    
def callback_distances(msg):
    global obstacle_distances
    obstacle_distances = msg.data
    
    return

def callback_base_link_pose(msg):
    global base_link_pose
    base_link_pose = msg.data
    
    return
    
def get_scan_base_coords():
    # Obtener la distancia entre la base de Takeshi y la base del láser scan.
    for i in range(10):
        try:
            trans = tfBuffer.lookup_transform('base_link', 'base_range_sensor_link', rospy.Time())
            return trans
        except:
            print ('waiting for tf')
            trans = 0

if __name__ == '__main__':

    rospy.init_node("obstacle_coords")
    rospy.Subscriber("/obstacle_detect_angles", Float32MultiArray, callback_angles)
    rospy.Subscriber("/obstacle_detect_distances", Float32MultiArray, callback_distances)
    rospy.Subscriber("/base_link_coords", Float32MultiArray, callback_base_link_pose)
    pub_coords_x = rospy.Publisher("/obstacle_coords_x", Float32MultiArray, queue_size=10)
    pub_coords_y = rospy.Publisher("/obstacle_coords_y", Float32MultiArray, queue_size=10)
    
    # Iniciar el árbol de transformaciones. Se necesita un delay para dar tiempo a cargar los
    # marcos de referencia.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1)
    
    loop = rospy.Rate(10)
    
    global obstacle_angles
    global obstacle_distances
    global base_link_pose
    
    base_to_scan = get_scan_base_coords().transform.translation.x
    
    while not rospy.is_shutdown():
        
        msg_coords_x = Float32MultiArray()
        msg_coords_x.data = [base_link_pose[0] + base_to_scan*np.cos(base_link_pose[2]) + obstacle_distances[i]*np.cos(base_link_pose[2] + obstacle_angles[i] - 2.1) for i in range(18)]
        pub_coords_x.publish(msg_coords_x)
        
        msg_coords_y = Float32MultiArray()
        msg_coords_y.data = [base_link_pose[1] + base_to_scan*np.sin(base_link_pose[2]) + obstacle_distances[i]*np.sin(base_link_pose[2] + obstacle_angles[i] - 2.1) for i in range(18)]
        pub_coords_y.publish(msg_coords_y)
        
        #print("Obstaculos detectados en:")
        #for i in range(18):
            #print("(",format(msg_coords_x.data[i], ".2f"),",", format(msg_coords_y.data[i], ".2f"),")")
        
        loop.sleep()

    
