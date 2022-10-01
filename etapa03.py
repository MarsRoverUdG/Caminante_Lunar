#!/usr/bin/env python3
from os import times
import rospy
import math
import tf
import tf2_ros
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray
#from control_algorithm import main as control
import numpy as np
from geometry_msgs.msg import Twist

#----------Object Detect----------------
import matplotlib.pyplot as plt
import numpy as np
import ros_numpy
import rospy
import tf
from gazebo_ros import gazebo_interface
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Pose, Quaternion ,TransformStamped

import sys
import cv2
import os

import moveit_commander
import moveit_msgs.msg

class RGBD():

    def __init__(self):
        self._br = tf.TransformBroadcaster()
        self._cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        self._h_image = None
        self._region = None
        self._h_min = 0
        self._h_max = 0
        self._xyz = [0, 0, 0]
        self._frame_name = None

    def _cloud_cb(self, msg):
        self._points_data = ros_numpy.numpify(msg)

        self._image_data = \
            self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

        hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = hsv_image[..., 0]

        self._region = \
            (self._h_image > self._h_min) & (self._h_image < self._h_max)

        if not np.any(self._region):
            return

        (y_idx, x_idx) = np.where(self._region)
        x = np.average(self._points_data['x'][y_idx, x_idx])
        y = np.average(self._points_data['y'][y_idx, x_idx])
        z = np.average(self._points_data['z'][y_idx, x_idx])
        self._xyz = [y, x, z]
        #self._xyz = [x, y, z]

        if self._frame_name is None:
            return

        self._br.sendTransform(
            (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
            self._frame_name,
            msg.header.frame_id)

    def get_image(self):
        return self._image_data

    def get_points(self):
        return self._points_data

    def get_h_image(self):
        return self._h_image

    def get_region(self):
        return self._region

    def get_xyz(self):
        return self._xyz

    def set_h(self, h_min, h_max):
        self._h_min = h_min
        self._h_max = h_max

    def set_coordinate_name(self, name):
        self._frame_name = name  
        
def get_objects_coords():
    
    # get_objects_coords() utiliza la cámara RGB para reconocer objetos y la nube de puntos para
    # encontrar la posición global de estos objetos con respecto a la cámara. La función regresa
    # una lista con las coordenadas de todos los objetos identificados, sin importar si son
    # objetos inválidos u objetos repetidos.
    
    # Rangos para la detección de objetos. Solo los objetos entre estos rangos se detectarán.
    r_min = 0.1
    r_max = 2.5    
    
    # Obtener imagen (480, 640, 3) y nube de puntos (480, 640).
    rgbd = RGBD()
    image = None
    points = None    
    # Esperar hasta que haya imágenes disponibles.
    while (image is None or points is None):
        image = rgbd.get_image()
        points = rgbd.get_points()
    cloud = np.nan_to_num(points['z'], nan = 10.0)
    
    # Detectar contornos. Modificar Canny para mejorar la detección de contornos.
    image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    contours, _  = cv2.findContours(cv2.Canny(image_gray, 25, 200), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Obtener las coordenadas de los objetos detectados y guardarlas en una lista.
    object_coords = list()
    for n in range(len(contours)):
        # Obtener píxeles dentro de los contornos.
        mask = cv2.drawContours(np.zeros((480,640)), contours, contourIdx=n, color=(255,255,255),thickness=-1).astype('bool')
        cloud_obj = cloud * mask
        # Eliminar medidas fuera del rango de detección.
        ind_obj = np.where(np.multiply(cloud_obj < r_max, cloud_obj > r_min))
        ind_obj = np.asarray(ind_obj)
                        
        # Encontrar el centro del objeto detectado.
        xyz = list()
        if ind_obj.shape[1]:
            for i in range(ind_obj.shape[1]):
                x = ind_obj[0][i]
                y = ind_obj[1][i]
                xyz.append([points['x'][x,y], points['y'][x,y], points['z'][x,y]])
            xyz = np.asarray(xyz)
            object_coords.append(xyz.mean(axis=0))
            
    # Encontrar la posición de los objetos con respecto al mapa.
    for n in range(len(object_coords)):
        broadcaster.sendTransform((object_coords[n][0],object_coords[n][1],object_coords[n][2]),(0,0,0,1), rospy.Time.now(), 'Object0',"head_rgbd_sensor_link")
        rospy.sleep(0.2)
        object_coords[n] = listener.lookupTransform('map','Object0',rospy.Time(0))[0]
    
    return np.asarray(object_coords)

def filter_objects_coords(object_coords, area, radio):
    
    # filter_objects_coords() recibe una lista con las posiciones de objetos identificados y
    # procede a eliminar aquellos objetos que se encuentran duplicados o los que se encuentren
    # fuera del área de interés. El área de interés está dada por la posición area (x,y) y un
    # radio.
    
    # Eliminar los objetos que se encuentran lejos del punto de interés.
    aux = list()
    for i in range(len(object_coords)):
        if (np.linalg.norm(object_coords[i][0:2] - area) < radio):
            aux.append(object_coords[i])
    
    object_coords = np.asarray(aux)
    
    # Eliminar objetos repetidos.
    aux = list()
    for i in range(len(object_coords)):
        flag = True
        for j in range(i+1, len(object_coords)):
            if (np.linalg.norm(object_coords[i][0:2] - object_coords[j][0:2]) < 0.1):
                flag = False
        if flag:
            aux.append(object_coords[i])
            
    filtered_object_coords = aux
    
    return filtered_object_coords

def arm_camera():
    print("Esperado a mover el brazo que estorba a la vision de la cámara")
    arm =  moveit_commander.MoveGroupCommander('arm')
    arm.set_named_target('go')
    arm.go(np.array((0, 0, 0.5*np.pi, -0.5*np.pi, 0, 0)))

    print("Esperando a inclinar la cabeza")
    head = moveit_commander.MoveGroupCommander('head')
    head.go(np.array((0,-.15*np.pi)))

def archivo(objects,f):
    for i in range(len(objects)):
        f.write("x: ")
        f.write(str(objects[i][0]))
        f.write("\t")
        f.write("y: ")
        f.write(str(objects[i][1]))
        f.write("\t")
        f.write("z: ")
        f.write(str(objects[i][2]))
        f.write("\n")

def object_detect(area, radio):
    while not rospy.is_shutdown():
        # Obtener coordenadas globales de todos los objetos publicados.
        object_coords = get_objects_coords()

        filtered_object_coords = filter_objects_coords(object_coords, area, radio)
        
        print('Objetos encontrados', filtered_object_coords,"\n\n")
        # Publicar TF de los objetos encontrados.
        for n in range(len(filtered_object_coords)):
            broadcaster.sendTransform((filtered_object_coords[n][0],filtered_object_coords[n][1],filtered_object_coords[n][2]),(0,0,0,1), rospy.Time.now(), 'Object' + str(n),"map")
            rospy.sleep(0.2)
        return filtered_object_coords
        #exit()


#--------Callback Functions-------------
def callback_obstacle_angles(msg):
    global obstacle_angles
    obstacle_angles = np.asarray(msg.data)

def callback_obstacle_distances(msg):
    global obstacle_distances
    obstacle_distances = np.asarray(msg.data)

def callback_base_pos(msg):
    global base_pos
    base_pos = np.asarray(msg.data)

def callback_station_pose(msg):
    global meta
    meta = np.asarray(msg.data)

def callback_time(msg):
    global time
    global clock
    time = int(str(msg.clock))//1000000000
    clock = True

#----------------Main--------------------
#def main():
def control(station):
    global obstacle_angles
    global obstacle_distances
    global base_pos
    global meta
    subs = rospy.Subscriber(station,Float32MultiArray,callback_station_pose)
    ## Control Parameters
    k_att = 1
    k_rep = 3
    kvx = .4 #* 1.5
    kvy = .4 #* 1.5
    kw = .1 #* 1.5
    h = 0.2
    d_max = 0.8
    pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
    loop = rospy.Rate(10)
    # Initial state
    n_sections = 24
    obstacle_angles = np.zeros((n_sections))
    obstacle_distances = np.zeros((n_sections))
    base_pos = np.asarray([2,2,2])
    meta = np.zeros((3))
    rospy.sleep(1)
    #bandera = True
    while not rospy.is_shutdown():
        ## CONTROL ALGORITHM ##
        #Force vectors calculus
        F_att = k_att*(meta[0:2] - base_pos[0:2])
        F_rep_x = k_rep*(obstacle_distances - d_max)*np.cos(obstacle_angles + base_pos[2])
        F_rep_y = k_rep*(obstacle_distances - d_max)*np.sin(obstacle_angles + base_pos[2])

        F_x = F_att[0] + np.sum(F_rep_x)
        F_y = F_att[1] + np.sum(F_rep_y)
        #Force normalization
        distance_left = math.sqrt(math.pow(meta[0]-base_pos[0],2) + pow(meta[1]-base_pos[1],2))
        normF_x = F_x if distance_left <=1.0 else F_x/math.sqrt(math.pow(F_x,2) + math.pow(F_y,2))
        normF_y = F_y if distance_left <=1.0 else F_y/math.sqrt(math.pow(F_x,2) + math.pow(F_y,2))
        # Gradient step
        x_d = base_pos[0] + h*normF_x 
        y_d = base_pos[1] + h*normF_y 
        #theta_d = math.atan2(y_d - base_pos[1],x_d - base_pos[0])
        theta_d = math.atan2(meta[1] - base_pos[1],meta[0] - base_pos[0])
        # If the step is close enough to the goal, then the reference is the goal
        if math.sqrt(math.pow(meta[0] - x_d,2) + math.pow(meta[1] - y_d,2)) <=0.5:
            x_d = meta[0]
            y_d = meta[1]
            theta_d = meta[2]
        print("Estacion: ",meta)
        print("Pose: ",base_pos)
        #print("Reference: ",x_d,", ",y_d,", ",theta_d)
        # Errors calculations
        error_x = x_d - base_pos[0]
        error_y = y_d - base_pos[1]
        error_w = math.atan2(math.sin(theta_d-base_pos[2]),math.cos(theta_d-base_pos[2]))
        # Message sending
        msg_cmd_vel = Twist()
        # Distance between the robot base and the goal
        dis_error = math.sqrt(math.pow(meta[0] - base_pos[0],2) + math.pow(meta[1] - base_pos[1],2))
        angle_error = math.atan2(math.sin(meta[2]-base_pos[2]),math.cos(meta[2]-base_pos[2]))
        # Condition to continue or stop the control proccess
        if dis_error >= 0.05 or abs(angle_error) >= 0.2:
            if dis_error < .05:
                kw = .3
            msg_cmd_vel.linear.x = kvx*(error_x*math.cos(base_pos[2]) + error_y*math.sin(base_pos[2]))
            msg_cmd_vel.linear.y = kvy*(-error_x*math.sin(base_pos[2]) + error_y*math.cos(base_pos[2]))
            msg_cmd_vel.angular.z = kw*error_w
        else:
            msg_cmd_vel.linear.x = 0
            msg_cmd_vel.linear.y = 0
            msg_cmd_vel.angular.z = 0
            subs.unregister()
            break
            #bandera = False
        pub_cmd_vel.publish(msg_cmd_vel)
        
        loop.sleep()
        
def main():
    global time
    global clock
    inicio, station_A, station_B, station_C, final = 0, 0, 0, 0, 0
    time = 0
    clock = False
    # Ros Node Config
    #rospy.init_node("etapa03")
    rospy.Subscriber("/obstacle_detect_angles",Float32MultiArray,callback_obstacle_angles)
    rospy.Subscriber("/obstacle_detect_distances",Float32MultiArray,callback_obstacle_distances)
    rospy.Subscriber("/base_link_coords", Float32MultiArray,callback_base_pos)
    rospy.Subscriber("/clock",Clock,callback_time)
    loop = rospy.Rate(1)
    while clock == False:
        print("Esperando subscripciones")
        loop.sleep()

    inicio = time
    
    control("Station_A")
    station_A = time
    print("Llegué a la estacion A en : ",station_A-inicio," segundos")        
    area = np.array((0, 1.21))
    radio = 0.75
    # Filtrar los objetos. Eliminar objetos repetidos y objetos fuera del área de interés.
    objects_a = object_detect(area, radio)
    rospy.sleep(1)

    control("Station_B")
    station_B = time
    print("Llegué a la estacion B en : ",station_B-station_A," segundos")
    area = np.array((-3, 4))
    radio = 0.75
    objects_b = object_detect(area, radio)
    rospy.sleep(1)

    #input("Ahora va al C")
    control("Station_C")
    station_C = time
    print("Llegué a la estacion C en : ",station_C-station_B," segundos")
    area = np.array((3.9, 5.6))
    radio = 0.75
    objects_c = (area, radio)
    objects_c = object_detect(area, radio)

    final = time
    print("Total: ",final-inicio," segundos")
    
    f = open('object_coords.txt','w')
    f.write("Objetos en la estacion A:\n")
    archivo(objects_a,f)
    f.write("\nObjetos en la estacion B:\n")
    archivo(objects_b,f)
    f.write("\nObjetos en la estacion C:\n")
    archivo(objects_c,f)
    f.close()

if __name__ == '__main__':
    try:
        arm_camera()
        rospy.init_node("etapa03")
        listener = tf.TransformListener()
        broadcaster= tf.TransformBroadcaster()
        rospy.sleep(1)
        main()
    except rospy.ROSInitException:
        pass