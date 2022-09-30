#!/usr/bin/env python3

''' SETUP '''

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
            

rospy.init_node("object_detect")
loop = rospy.Rate(10)

listener = tf.TransformListener()
broadcaster= tf.TransformBroadcaster()
rospy.sleep(1)

''' FIN SETUP '''

''' Inclinar cabeza y quitar brazo '''

# Comandos para inclinar la cámara y esconder el brazo. Tardan mucho en hacer efecto.
arm =  moveit_commander.MoveGroupCommander('arm')
arm.set_named_target('go')
arm.go(np.array((0, 0, 0.5*np.pi, -0.5*np.pi, 0, 0)))

head = moveit_commander.MoveGroupCommander('head')
head.go(np.array((0,-.15*np.pi)))

''' Fin Inclinar cabeza y quitar brazo '''

if __name__ == '__main__':
    while not rospy.is_shutdown():
        
        # Obtener coordenadas globales de todos los objetos publicados.
        object_coords = get_objects_coords()
        
        # Filtrar los objetos. Eliminar objetos repetidos y objetos fuera del área de interés.
        area = np.array((-3.0, 4.0))
        radio = 0.75
        filtered_object_coords = filter_objects_coords(object_coords, area, radio)
        
        print('Objetos encontrados', filtered_object_coords)
        
        # Publicar TF de los objetos encontrados.
        for n in range(len(filtered_object_coords)):
            broadcaster.sendTransform((filtered_object_coords[n][0],filtered_object_coords[n][1],filtered_object_coords[n][2]),(0,0,0,1), rospy.Time.now(), 'Object' + str(n),"map")
            rospy.sleep(0.2)
            
        exit()
                    
        loop.sleep()
