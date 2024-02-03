#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import ros_numpy
import numpy as np
import math


def handle_callback(pCloud: PointCloud2):
    raw_data = ros_numpy.point_cloud2.pointcloud2_to_array(pCloud)
    if(roll!=0 or pitch !=0 or yaw!=0):

        data = np.concatenate(raw_data)
        data = np.stack([data['x'], data['y'], data['z'], data['intensity'], data['ring'], data['timestamp']], axis=-1)
        
        data = data@M
        
        data = data[data[:,3]>=0]
        data = np.array(list(map(tuple,data)),dtype=raw_data.dtype)

        cloud = ros_numpy.point_cloud2.array_to_pointcloud2(data,frame_id='velodyne')
        cloud.header.stamp = pCloud.header.stamp
        pub.publish(cloud)

    else:
        data = np.concatenate(raw_data)
        cloud = ros_numpy.point_cloud2.array_to_pointcloud2(data,frame_id='velodyne')
        cloud.header.stamp = pCloud.header.stamp
        pub.publish(cloud)

if __name__ == '__main__':

    roll=(-2)*np.pi/180
    pitch=(-3)*np.pi/180
    yaw=(-25)*np.pi/180

    # Mr = np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
    # Mp = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
    # My = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    Mr = np.array([[1,0,0,0,0,0],[0,np.cos(roll),-np.sin(roll),0,0,0],[0,np.sin(roll),np.cos(roll),0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
    Mp = np.array([[np.cos(pitch),0,np.sin(pitch),0,0,0],[0,1,0,0,0,0],[-np.sin(pitch),0,np.cos(pitch),0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
    My = np.array([[np.cos(yaw),-np.sin(yaw),0,0,0,0],[np.sin(yaw),np.cos(yaw),0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
    M = Mr@Mp@My


    rospy.init_node("velodyne_transformer")
    pub = rospy.Publisher("/velodyne_points",
                          PointCloud2, queue_size=1)
    sub = rospy.Subscriber("/rslidar_points", PointCloud2,
                           callback=handle_callback, queue_size=1, buff_size=2**30)

    rospy.loginfo("Node has been started.")

    rospy.spin()