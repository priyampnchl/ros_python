#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class poseReporterClass():

    PUB_FREQ = 2  # Hz This rate is bottlenecking the turtle tbh, it works way better with a higher rate

    def __init__(self):
        # print("__init__ is working")

        self.target_pose = Pose()
        self.current_pose = Pose()
        self.driveinfo = Twist()

        # initialise the node "pose_reporter"
        rospy.init_node('pose_reporter', anonymous=False)
        print("node pose_reporter created")

        self.sub = rospy.Subscriber(
            '/turtle1/destination', Pose, self.parse_target)
        print("subscribed to /turtle1/destination")

        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.parse_current)
        print("subscribed to /turtle1/pose")

        self.driverinfopublisher = rospy.Publisher(
            '/turtle1/driverinfo', Twist, queue_size=10)
        self.pub_rate = rospy.Rate(self.PUB_FREQ)  # define the rate
        print("publishing to /turtle1/driverinfo")

        # self.log_rate = rospy.Rate(self.LOG_FREQ)

    def parse_target(self, data):
        self.target_pose = data
        # print("parse_target says hello")
        self.target_pose.x = data.x
        self.target_pose.y = data.y

    def parse_current(self, data):
        # print("parse_current says hello")
        self.current_pose = data
        self.current_pose.x = data.x
        self.current_pose.y = data.y

    def compute_distance(self):
        self.distance = math.sqrt(
            ((self.target_pose.y-self.current_pose.y)**2 + (self.target_pose.x-self.current_pose.x)**2))

    def compute_angle_err(self):
        self.current_theta = self.current_pose.theta
        # print(self.current_pose.theta)

        self.goal_theta = math.atan2(
            (self.target_pose.y-self.current_pose.y), (self.target_pose.x-self.current_pose.x))

        self.orientationerror = self.goal_theta-self.current_theta
        # print(self.orientationerror)

    def publisher_function(self):
        print("publisherFunction is running")  # some comfort
        while not rospy.is_shutdown():
            self.compute_distance()
            self.compute_angle_err()
            self.current_theta_deg = self.current_pose.theta * 57.2958#radians to degrees
            self.orientationerror_deg = self.orientationerror * 57.2958

            rospy.loginfo("X:%.3f metres, Y:%.3f metres, Distance:%.3f metres, Current Orientation:%.3f degrees, Orientation Error:%.3f degrees",self.current_pose.x,  self.current_pose.y,  self.distance,    self.current_theta_deg, self.orientationerror_deg)

            #assigning signals to be published to the Twist message
            self.driveinfo.linear.x = self.current_pose.x
            self.driveinfo.linear.y = self.current_pose.y
            self.driveinfo.linear.z = self.distance
            self.driveinfo.angular.x = self.current_pose.theta
            self.driveinfo.angular.y = self.orientationerror

            self.driverinfopublisher.publish(self.driveinfo)  # publish target
            self.pub_rate.sleep()


if __name__ == '__main__':
    posRepObj = poseReporterClass()#instantiating a class object
    try:
        posRepObj.publisher_function()#publish driveinfo

    except rospy.ROSInterruptException:
        pass
