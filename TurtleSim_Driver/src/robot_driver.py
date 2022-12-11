#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

#if you are reading this it is because my code did not run and I am a terrible programmer, apologies

class DriverBhaya():

    FREQ=20#hz

    def __init__(self):
        
        self.pose=Pose()
        self.velocity_to_publish=Twist()
        self.driveinfoinput=Twist()
        self.tolerance=0.1
        
        rospy.init_node('robot_driver',anonymous=False)     #initialise the node "robot_driver"
        print("node robot_driver created")
        
        self.sub=rospy.Subscriber('/turtle1/driverinfo', Twist, self.parse_driverinfo)
        print("subscribed to turtle1/driverinfo")
        
        self.driver_publisher=rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #feedback: Why instantiate now? only instantiate when you have something to publish
        print("publishing to turtle1/cmd_vel")

    def parse_driverinfo(self,data):
        self.driveinfoinput=data
                
        if not self.driveinfoinput.linear.z < self.tolerance: #while target not reached
            print(self.driveinfoinput.angular.y)
            
            if self.driveinfoinput.angular.y > 0:#target to the left of turtle
                self.velocity_to_publish.linear.x=0.065*self.driveinfoinput.linear.z#proportional linear velocity control
                self.velocity_to_publish.angular.z=0.35*self.driveinfoinput.angular.y#proportional angular velocity control
                print("###########    NAVIGATING LEFT    #############")

            elif self.driveinfoinput.angular.y <0:#target to the right of turtle
                self.velocity_to_publish.linear.x=0.065*self.driveinfoinput.linear.z#proportional linear velocity control
                self.velocity_to_publish.angular.z=-0.35*abs(self.driveinfoinput.angular.y)#proportional angular velocity control, absolute value because dont want double negative
                print("###########    NAVIGATING RIGHT   ############")

            else:
                pass
                # print("angle error zero")#super duper edge case for debugging
                
        else:
            self.velocity_to_publish.linear.x=0#stop
            self.velocity_to_publish.angular.z=0#stop
            print("##########    DESTINATION REACHED    ##########")

    def publish_velocities(self):
        self.rate=rospy.Rate(self.FREQ)
        self.driver_publisher.publish(self.velocity_to_publish)
        self.rate.sleep()

if __name__=='__main__':
    driver=DriverBhaya()
    while not rospy.is_shutdown():
        try:
            driver.publish_velocities()
        except rospy.ROSInterruptException():
            pass