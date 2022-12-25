#!/usr/bin/env python

import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose

class LidarBaby():

    PUB_FREQ=10# Hz
    CCMVENT_DIST=1# 1 metres, this is "l" value from the assignment
    MAX_ERROR=0.3# 0.3 metres
    KP=0.2# Proportional Coefficient

    #Information to translate lidar frame (/base_scan) to robot frame (/base_link)
    X_L=-0.032# Metres
    Y_L=0# Metres
    Theta_L=0# Radians

    def __init__(self):
        
        print("init is working")

        self.telemetry_data=Pose()

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

        self.error=self.min_distance-self.CCMVENT_DIST
        
        #implementing eroor funciton as given in the assignment prompt
        if self.error >= self.MAX_ERROR:
            self.e_bar=1
        elif self.MAX_ERROR > self.error > -self.MAX_ERROR : 
            self.e_bar = self.error / self.MAX_ERROR
        elif -self.MAX_ERROR >= self.error:
            self.e_bar = -1
        else:
            self.e_bar=float('NaN')

        try:
            self.first_bearing_index=self.bearing_index[0]#just pass the first min value
            self.bearing=(self.first_bearing_index*msg.angle_increment+msg.angle_min)*180.0/np.pi#convert the bearing to usable degree value

            self.telemetry_data.position.x=self.min_distance#minimum distance to obstacle
            self.telemetry_data.position.y=self.error
            self.telemetry_data.position.z=self.e_bar
            self.telemetry_data.orientation.x=self.KP*self.e_bar
            self.telemetry_data.orientation.y=self.KP*(1-abs(self.e_bar))
            self.telemetry_data.orientation.z=math.radians(self.bearing)#bearing to minimum distance 

        except:
            self.bearing=float('NaN')
            self.min_distance=float('NaN')
            self.telemetry_data.position.x=self.min_distance#minimum distance to obstacle
            self.telemetry_data.position.y=self.error
            self.telemetry_data.position.z=self.e_bar
            self.telemetry_data.orientation.x=self.KP*self.e_bar
            self.telemetry_data.orientation.y=self.KP*(1-abs(self.e_bar))
            self.telemetry_data.orientation.z=self.bearing#bearing to minimum distance 


    def telemetry_publisher(self):
        self.telemetry_publisher_obj = rospy.Publisher('/sensed_object', Pose, queue_size=10)
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