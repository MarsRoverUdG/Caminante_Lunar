#!/usr/bin/env python3

# Nodo que publica la pose del base_link de Takeshi con respecto al marco de referencia 'map'.
# La pose se publica en el tópico /base_link_coords en un mensaje tipo Float32MultiArray
# con un formato [x, y, theta_rad].
 
import rospy
import tf
import tf2_ros
<<<<<<< Updated upstream
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import PoseStamped
    
def get_base_link_pose():
    # get_base_link_pose() regresa un lista con la pose de la base del base_link con
    # respecto al marco de referencia global en la forma [x, y, theta] con theta en radianes.
    global time
    trans = get_base_link_coords()
    time = trans.header.stamp.to_sec()
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    (_, _, theta) = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

    return list([x, y, theta])
=======
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, PoseStamped

#def callback_meta(msg):
#    global meta
#    meta = PoseStamped_to_pose(msg)
#    print('Pose meta:  ', PoseStamped_to_pose(msg))
#    print('Pose robot: ', get_base_link_pose())
    
#def PoseStamped_to_pose(msg):
#    # PoseStamped_to_pose2d() convierte un mensaje tipo PoseStamped a una pose 2d en un arreglo
#    # de Numpy con la forma (x,y,theta) con theta en radianes.
#    
#    x = msg.pose.position.x
#    y = msg.pose.position.y
#    (_, _, theta) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
#
#    return np.array([x, y, theta])
>>>>>>> Stashed changes

def get_base_link_coords():
    for i in range(10):
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            return trans
        except:
            print ('waiting for tf')
            trans = 0
    
def get_base_link_pose():
    # get_base_link_pose() regresa un lista con la pose de la base del base_link con
    # respecto al marco de referencia global en la forma [x, y, theta] con theta en radianes.
    
    trans = get_base_link_coords()
    
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    (_, _, theta) = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

    return list([x, y, theta])

def PoseStamped_to_pose(msg):
    # PoseStamped_to_pose2d() convierte un mensaje tipo PoseStamped a una pose 2d en un arreglo
    # de Numpy con la forma (x,y,theta) con theta en radianes.
    global meta_coords
    x = msg.pose.position.x
    y = msg.pose.position.y
    (_, _, theta) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    meta_coords =  list([x, y, theta])

if __name__ == '__main__':
<<<<<<< Updated upstream
    global meta_coords
    global time
    time = 0
    rospy.init_node("base_link_coords")
    rospy.Subscriber("/meta_competencia", PoseStamped, PoseStamped_to_pose)
    pub_coords = rospy.Publisher("/base_link_coords", Float32MultiArray, queue_size=10)
    pub_meta_coords = rospy.Publisher("/meta_coords", Float32MultiArray, queue_size=10)
    pub_time = rospy.Publisher("/time",Int32,queue_size=10)
=======
    #global meta
    rospy.init_node("base_link_coords")
    #rospy.Subscriber("/meta_competencia", PoseStamped,callback_meta)
    pub_coords = rospy.Publisher("/base_link_coords", Float32MultiArray, queue_size=10)
    #pub_meta_coords = rospy.Publisher("/meta_competencia_x_y_theta", Float32MultiArray, queue_size=10)
>>>>>>> Stashed changes
    # Iniciar el árbol de transformaciones. Se necesita un delay para dar tiempo a cargar los
    # marcos de referencia.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1)
    
    loop = rospy.Rate(10)
    meta_coords = False
    while not rospy.is_shutdown():
<<<<<<< Updated upstream
        if meta_coords != False:
            base_coords = get_base_link_pose()
            msg_time = Int32()
            msg_base_coords = Float32MultiArray()
            msg_meta_coords = Float32MultiArray()
            msg_time = int(time)
            msg_base_coords.data = base_coords
            msg_meta_coords.data = meta_coords
            pub_coords.publish(msg_base_coords)
            pub_meta_coords.publish(msg_meta_coords)
            pub_time.publish(msg_time)
            print("Pose Meta: ",meta_coords)
            print("Pose Base: ",base_coords)
=======
        
        coords = Float32MultiArray()
        coords.data = get_base_link_pose()
        pub_coords.publish(coords)
        print(coords.data)
        #meta_coords = Float32MultiArray()
        #meta_coords.data = meta
        #pub_meta_coords.publish(meta_coords)
        
>>>>>>> Stashed changes
        loop.sleep()

    
