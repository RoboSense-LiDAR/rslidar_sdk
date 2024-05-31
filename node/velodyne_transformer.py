#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import ros_numpy
import numpy as np
import math
import time


def handle_callback(pCloud: PointCloud2):
    start = time.time()
    raw_data = ros_numpy.point_cloud2.pointcloud2_to_array(pCloud)
        
    data = np.concatenate(raw_data)
    
    data = np.stack([data['x'], data['y'], data['z'], data['intensity'], data['ring'], data['timestamp']], axis=-1)
    
    data[:,5] = data[:,5]-data[0,5]
    
    
    if(roll!=0 or pitch !=0 or yaw!=0):
        data = data@M
   

    data = data[~np.isnan(data).any(axis=1)]
    


    data = np.array(list(map(tuple,data)),dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('ring', '<u2'), ('time', '<f4')])
    # rospy.loginfo(data[-1])
    
    
    cloud = ros_numpy.point_cloud2.array_to_pointcloud2(data,frame_id='velodyne')
    cloud.header.stamp = pCloud.header.stamp
    
    # cloud.is_dense=True
    pub.publish(cloud)
    rospy.logwarn(time.time()-start)

if __name__ == '__main__':

    roll=-1.2
    pitch=-2.52
    yaw=0

    roll=roll*np.pi/180
    pitch=pitch*np.pi/180
    yaw=yaw*np.pi/180

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