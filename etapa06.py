#!/usr/bin/env python3
from os import times
import rospy
import math
import tf
import tf2_ros as tf2
import matplotlib.pyplot as plt
import numpy as np
import ros_numpy
import sys
import cv2
import os
import math
import moveit_commander
import moveit_msgs.msg
import tmc_control_msgs.msg
import trajectory_msgs.msg

from moveit_commander.conversions import pose_to_list
from gazebo_ros import gazebo_interface
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, WrenchStamped, TransformStamped, Pose, Quaternion
from sensor_msgs.msg import LaserScan, PointCloud2

#--------Callback Functions-------------

def callback_base_pos(msg):
    global base_pos
    base_pos = np.asarray(msg.data)

#--------Defined Classes-------------

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

class GRIPPER():
    def __init__(self):
        self._grip_cmd_pub = rospy.Publisher('/hsrb/gripper_controller/command',
                               trajectory_msgs.msg.JointTrajectory, queue_size=100)
        self._grip_cmd_force = rospy.Publisher('/hsrb/gripper_controller/grasp/goal',
        			tmc_control_msgs.msg.GripperApplyEffortActionGoal, queue_size=100)
        			
        self._joint_name = "hand_motor_joint"
        self._position = 0.5
        self._velocity = 0.5
        self._effort = 0.0
        self._duration = 1

    def _manipulate_gripper(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = [self._joint_name]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self._position]
        p.velocities = [self._velocity]
        p.accelerations = []
        p.effort = [self._effort]
        p.time_from_start = rospy.Duration(self._duration)
        traj.points = [p]
        self._grip_cmd_pub.publish(traj)
        
    def _apply_force(self):
        app_force = tmc_control_msgs.msg.GripperApplyEffortActionGoal()
        app_force.goal.effort = -0.5
        self._grip_cmd_force.publish(app_force)
        
    def change_velocity(self, newVel):
        self._velocity = newVel
    
    def open(self):
        self._position = 1.23
        self._effort = 0
        self._manipulate_gripper()

    def steady(self):
        self._position = -0.82
        self._effort = -0.3
        self._manipulate_gripper()
        
    def close(self):
        self._position = -0.82
        self._effort = -0.3
        self._manipulate_gripper()
        self._apply_force()
        rospy.sleep(0.8)

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

#--------Defined Functions-------------

def get_objects_coords():
    
    # get_objects_coords() utiliza la cámara RGB para reconocer objetos y la nube de puntos para
    # encontrar la posición global de estos objetos con respecto a la cámara. La función regresa
    # una lista con las coordenadas de todos los objetos identificados, sin importar si son
    # objetos inválidos u objetos repetidos.
    
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    rospy.sleep(0.5)
    
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

def object_detect(area, radio):
    
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    rospy.sleep(0.5)
    
    # Obtener coordenadas globales de todos los objetos publicados.
    object_coords = get_objects_coords()
    
    filtered_object_coords = filter_objects_coords(object_coords, area, radio)
    
    # Publicar TF de los objetos encontrados.
    for n in range(len(filtered_object_coords)):
        broadcaster.sendTransform((filtered_object_coords[n][0],filtered_object_coords[n][1],filtered_object_coords[n][2]),(0,0,0,1), rospy.Time.now(), 'Object' + str(n),"map")
        rospy.sleep(0.2)
    return filtered_object_coords

def robot_scan_pose():
    arm =  moveit_commander.MoveGroupCommander('arm')
    arm.set_named_target('go')
    arm.go(np.array((0, 0, 0.5*np.pi, -0.5*np.pi, 0, 0)))

    head = moveit_commander.MoveGroupCommander('head')
    head.go(np.array((0,-.15*np.pi)))

def move_base(meta_pos):

    global base_pos

    rospy.Subscriber("/base_link_coords", Float32MultiArray,callback_base_pos)
    pub_coords = rospy.Publisher("/meta_coords", Float32MultiArray, queue_size=10)
    loop = rospy.Rate(10)
    rospy.wait_for_message('/base_link_coords', Float32MultiArray, timeout=10000)

    print('Moviendo base. x: ', meta_pos[0], ' y: ', meta_pos[1], ' theta: ', meta_pos[2])	
    meta_coords = Float32MultiArray()
    meta_coords.data = meta_pos
	
    dis_error = math.sqrt(math.pow(meta_pos[0]-base_pos[0],2)+math.pow(meta_pos[1] - base_pos[1], 2))
    angle_error = math.atan2(math.sin(meta_pos[2]-base_pos[2]), math.cos(meta_pos[2]-base_pos[2]))
    while dis_error >= 0.01 or abs(angle_error) >= 0.01:
        dis_error = math.sqrt(math.pow(meta_pos[0]-base_pos[0],2)+math.pow(meta_pos[1] - base_pos[1], 2))
        angle_error = math.atan2(math.sin(meta_pos[2]-base_pos[2]), math.cos(meta_pos[2]-base_pos[2]))
        pub_coords.publish(meta_coords)
        loop.sleep()

    print('Base en posición. x: ', "{:.2f}".format(base_pos[0]), ' y: ', "{:.2f}".format(base_pos[1]), ' theta: ', "{:.2f}".format(base_pos[2]))

def scan_position(area, radio):
    # Función para escanear una posición en busca de objetos. area = [x_global, y_global] tiene las coordenadas
    # de la posición que se va a escanear y radio da el radio de búsqueda. Se regresa una lista 'objects' con
    # las coordenadas x, y, z globales de los objetos encontrados.
    # 1. Se posiciona la cámara y se quita el brazo para que no estorbe a la visión.
    # 2. Se toma la información de la cámara de profundidad y se detectan los objetos.

    robot_scan_pose()
    objects = object_detect(area, radio)
    return objects

#----------------Main--------------------

def main():

    print('MISIÓN LUNAR - UDEG SPACE')

    rospy.init_node("etapa06")
    rospy.Subscriber("/base_link_coords", Float32MultiArray,callback_base_pos)
    pub_coords = rospy.Publisher("/meta_coords", Float32MultiArray, queue_size=10)
    loop = rospy.Rate(10)
    
    # Comandos de inicialización de moveit
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # 1. Moverse a las rocas
    meta_pos = [0.0, 0.1, 1.57]
    move_base(meta_pos)

    # 2. Escanear una zona
    area = np.array((0, 1.21))
    radio = 1.0
    objects = scan_position(area, radio)
    print(objects)

    # 4. Prueba gripper
    grip = GRIPPER()
    print("abriendo pinza")
    grip.open()
    print("cerrando pinza")
    grip.close()

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
