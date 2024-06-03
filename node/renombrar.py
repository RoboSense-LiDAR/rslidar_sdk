#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import ros_numpy
from sensor_msgs.msg import Imu
import numpy as np
import math




def handle_callback(data: Imu):
    global head
    if head != None:
        msg = Imu()
        msg.header = head
        msg.orientation = data.orientation
        msg.linear_acceleration.z=-9.8
        pub.publish(msg)
        rospy.loginfo("a")

def handle_callback2(data: PointCloud2):
    global head
    head = data.header
   
   

if __name__ == '__main__':

    head=None
    rospy.init_node("imu_transformer")
    pub = rospy.Publisher("/vectornav/IMU", Imu, queue_size=1)
    rospy.Subscriber("/can/IMU", Imu, callback=handle_callback, queue_size=1, buff_size=2**30)
    rospy.Subscriber("/velodyne_points", PointCloud2, callback=handle_callback2, queue_size=1, buff_size=2**30)
    rospy.Subscriber("/rslidar_points", PointCloud2, callback=handle_callback2, queue_size=1, buff_size=2**30)

    rospy.loginfo("Node has been started.")

    rospy.spin()