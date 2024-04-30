#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import ros_numpy
from sensor_msgs.msg import Imu
import numpy as np
import math

suma=0
n=0

accel_suma = np.array([0.,0.,0.])


def handle_callback(data: Imu):
    msg = Imu()

    msg.orientation.x = data.orientation.x
    msg.orientation.y = data.orientation.y
    msg.orientation.z = data.orientation.z
    msg.orientation.w = data.orientation.w

    gyro = np.array([data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z])
    accel = np.array([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z])

    global suma,n,accel_suma
    suma +=np.linalg.norm(accel)
    accel_suma+=accel
    n+=1
    # rospy.loginfo(accel_suma/n)
    v = accel_suma/n
    g = np.array([0,-0.981])
    roll2 = np.arccos(v[[1,2]]@g/(np.linalg.norm(v[[1,2]])*np.linalg.norm(g)))
    pitch2 = np.arccos(v[[0,2]]@g/(np.linalg.norm(v[[0,2]])*np.linalg.norm(g)))


    print([roll2*180/np.pi,pitch2*180/np.pi])
    # print(M@accel)



    gyro = gyro/10
    gyro = M@gyro
    accel = M@accel

    msg.angular_velocity.x = gyro[0]
    msg.angular_velocity.y = gyro[1]
    msg.angular_velocity.z = gyro[2]

    msg.linear_acceleration.x =  accel[0]
    msg.linear_acceleration.y =  accel[1]
    msg.linear_acceleration.z =  accel[2]

    msg.header.stamp = data.header.stamp
    msg.header.frame_id = 'velodyne'


    pub.publish(msg)

   
   

if __name__ == '__main__':
    
    roll=2.23
    pitch=5
    yaw=0

    roll=roll*np.pi/180
    pitch=pitch*np.pi/180
    yaw=yaw*np.pi/180

    Mr = np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
    Mp = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
    My = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    M = Mr@Mp@My


    rospy.init_node("imu_transformer")
    pub = rospy.Publisher("/can/IMU", Imu, queue_size=1)
    sub = rospy.Subscriber("/IMU", Imu, callback=handle_callback, queue_size=1, buff_size=2**30)

    rospy.loginfo("Node has been started.")

    rospy.spin()