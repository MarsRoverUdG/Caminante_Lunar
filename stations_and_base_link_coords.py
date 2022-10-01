#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

def get_base_link_pose():
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

def callback_base_link_pose(msg):
    global base_link_pose
    x = msg.pose.position.x
    y = msg.pose.position.y
    (_, _, theta) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    base_link_pose =  list([x, y, theta])

def msg_station_pose(station_pose):
    msg_station = Float32MultiArray()
    msg_station.data = station_pose
    return msg_station


if __name__ == '__main__':
#def main():
    global base_link_pose
    base_link_pose = 0
    '''math.pi/2+.2'''
    station_A = list([0.0, 0.0, (math.pi/2) + .2])
    #station_A = list([-4.0, -4.0, math.pi/2])
    station_B = list([-2.0, 3.0, 3*math.pi/4])
    station_C = list([2.1, 5.7, -0.2])
    inicio = list([0.0, 0.0, 0.0])

    rospy.init_node("stations")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tfBuffer.lookup_transform
    rospy.sleep(1)
    rospy.Subscriber("/global_pose",PoseStamped, callback_base_link_pose)
    pub_station_A = rospy.Publisher("Station_A",Float32MultiArray, queue_size=10)
    pub_station_B = rospy.Publisher("Station_B",Float32MultiArray, queue_size=10)
    pub_station_C = rospy.Publisher("Station_C",Float32MultiArray, queue_size=10)
    pub_inicio = rospy.Publisher("inicio",Float32MultiArray, queue_size=10)
    pub_base_coords = rospy.Publisher("/base_link_coords",Float32MultiArray,queue_size=10)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        station_A_pose = msg_station_pose(station_A)
        pub_station_A.publish(station_A_pose)
        station_B_pose = msg_station_pose(station_B)
        pub_station_B.publish(station_B_pose)
        station_C_pose = msg_station_pose(station_C)
        pub_station_C.publish(station_C_pose)
        inicio_pose = msg_station_pose(inicio)
        pub_inicio.publish(inicio_pose)


        msg_base_coords = Float32MultiArray()
        #msg_base_coords.data = base_link_pose
        msg_base_coords.data = get_base_link_pose()
        pub_base_coords.publish(msg_base_coords)

        print("Pose base: ",base_link_pose)
        print("Station A",station_A)
        print("Station B",station_B)
        print("Station C",station_C,)
        print("inicio ",inicio,"\n")
        loop.sleep()


'''if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass'''