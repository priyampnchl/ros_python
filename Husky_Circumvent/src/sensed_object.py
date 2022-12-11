#!/usr/bin/env python

## Subscribes to topic /scan of type LaserScan and publish the minimum
## distance to an obstacle and their angle(s) relative to the scanner's
## reference frame

import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

class LidarBaby():

    PUB_FREQ=10#Hz

    def __init__(self):
        
        print("init is working")

        self.telemetry_data=Pose2D()

        rospy.init_node('sensed_object', anonymous=False)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.compute_telemetry)
        print("subscribed to /scan")

    def compute_telemetry(self, msg):
         
        self.lidar_data=msg.ranges
        self.np_lidar_ranges=np.array(self.lidar_data)#create numpy array to be able to use the magic of numpy
        # print(self.np_lidar_data)

        self.np_lidar_ranges[self.np_lidar_ranges > msg.range_max] = float('NaN')
        self.np_lidar_ranges[self.np_lidar_ranges < msg.range_min] = float('NaN')
        self.min_distance=np.nanmin(self.np_lidar_ranges)
        self.bearing_index=np.reshape( np.argwhere(self.np_lidar_ranges == self.min_distance) , -1)

        try:
            self.first_bearing_index=self.bearing_index[0]#just pass the first min value
            self.bearing=(self.first_bearing_index*msg.angle_increment+msg.angle_min)*180.0/np.pi#convert the bearing to usable degree value
            self.telemetry_data.x=self.min_distance#minimum distance to obstacle
            self.telemetry_data.theta=self.bearing#bearing to minimum distance 

        except:
            self.bearing=float('NaN')
            self.min_distance=float('NaN')
            self.telemetry_data.x=self.min_distance
            self.telemetry_data.theta=self.bearing


    def telemetry_publisher(self):
        self.telemetry_publisher_obj = rospy.Publisher('/sensed_object', Pose2D, queue_size=10)
        self.pub_rate = rospy.Rate(self.PUB_FREQ)
        print("publishing to /sensed_object")
        while not rospy.is_shutdown():
            self.telemetry_publisher_obj.publish(self.telemetry_data)
            self.pub_rate.sleep()
            

if __name__ == '__main__':
    lidarbabyobj = LidarBaby()
    try:
        lidarbabyobj.telemetry_publisher()
    except rospy.ROSInterruptException():
        pass