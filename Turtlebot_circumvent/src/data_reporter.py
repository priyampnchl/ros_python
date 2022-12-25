#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

class DataReporter():

    PUB_FREQ=2# Hz

    def __init__(self):
        
        print("init is working")

        self.telemetry_data=Pose()
        self.velocity_data=Twist()

        rospy.init_node('data_reporter', anonymous=False)
        self.sub = rospy.Subscriber('/sensed_object', Pose, self.clbk)# subscribe to sensed object to get d, e, ebar etc.
        print("subscribed to /sensed_object")

        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_clbk)# subscribe to cmd_vel to get linear and angular velocities v and w(omega)
        print("subscribed to /cmd_vel")


    def clbk(self,msg): #callback for sensed object data
        self.telemetry_data=msg

    def vel_clbk(self,msg): #callback for cmd_vel
        self.velocity_data=msg

    def telemetry_publisher(self):
        self.telemetry_publisher_obj = rospy.Publisher('/data_reporter_log', Pose, queue_size=10)
        self.pub_rate = rospy.Rate(self.PUB_FREQ)
        print("publishing to /data_reporter_log")
        while not rospy.is_shutdown():
            rospy.loginfo("Dist to obstacle:    " + str(self.telemetry_data.position.x)+"   metres")
            rospy.loginfo("Error:               " + str(self.telemetry_data.position.y)+"   metres")
            rospy.loginfo("e_bar:               " + str(self.telemetry_data.position.z)+"   metres")
            rospy.loginfo("Alpha:               " + str(math.degrees(self.telemetry_data.orientation.z))+"  degrees")
            rospy.loginfo("Velocity of L in A:  " + "[" + str(self.telemetry_data.orientation.x)+", "+ str(self.telemetry_data.orientation.y) + "]")
            rospy.loginfo("Linear Velocity:     " + str(self.velocity_data.linear.x)+"  metres/sec")
            rospy.loginfo("Angular Velocity:    " + str(math.degrees(self.velocity_data.angular.z))+"   degrees/sec")
            rospy.loginfo("###############################################################")
            self.telemetry_publisher_obj.publish(self.telemetry_data)
            self.pub_rate.sleep()
            
if __name__ == '__main__':
    datareporterobj = DataReporter()
    try:
        datareporterobj.telemetry_publisher()
    except rospy.ROSInterruptException():
        pass