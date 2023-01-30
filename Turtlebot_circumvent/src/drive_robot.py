#!/usr/bin/env python

import math
import numpy as np
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

class Driver():

    PUB_FREQ=10# Hz
    
    X_L=-0.032# Metres
    Y_L=0# Metres
    Theta_L=0# Radians


    def __init__(self):
        
        self.obstacle_found=False

        self.telemetry=Pose()
        self.drive_command=Twist() # #initialise drive_command object of type Twist
        self.sub=rospy.Subscriber('/sensed_object', Pose, self.parse_telemetry) #initialise subscriber and subscribe to /sensed_object
        
        rospy.init_node('drive_robot', anonymous=False)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)#initialise the publisher
        self.pub_rate = rospy.Rate(self.PUB_FREQ)
        print("publishing to /cmd_vel")


    def parse_telemetry(self,msg):
        self.telemetry=msg

    def stop(self):#it's in the name
        self.drive_command.linear.x=0
        self.drive_command.angular.z=0

    def detect_obstacle(self):
        self.obstacle_found=False
        if (math.isnan(self.telemetry.position.x) and math.isnan(self.telemetry.orientation.z) and math.isnan(self.telemetry.position.y)): #check if lidar is sending nan
            self.stop()#stop if no obstacles, duh            
            print("NO OBSTACLE FOUND: NaN DETECTED")

        else: #if lidar data is non-NaN
            self.obstacle_found=True
            print("OBSTACLE FOUND")

    def drive(self):

        self.detect_obstacle()

        if self.obstacle_found:
            self.compute_drive_commands()
        else:
            self.stop()
            

    def compute_drive_commands(self):
        d       =self.telemetry.position.x
        e       =self.telemetry.position.y
        e_bar   =self.telemetry.position.z
        AVL_x   =self.telemetry.orientation.x
        AVL_y   =self.telemetry.orientation.y
        alpha   =math.degrees(-1*self.telemetry.orientation.z)#times minus 1 because it was behaving opposite the way it should, quick fix instead of transposing
        xL=self.X_L
        yL=self.Y_L
        theta=self.Theta_L

        #implementing the control funciton
        self.matrix=np.array([[math.cos(math.radians(theta+alpha))+((yL/xL)*math.sin(math.radians(theta+alpha))),(-(math.sin(math.radians(theta+alpha)))+(yL/xL)*math.cos(math.radians(theta+alpha)))],[(1/xL)*math.sin(math.radians(theta+alpha)),(1/xL)*math.cos(math.radians(theta+alpha))]])
        
        self.AVL_matrix=np.array([[AVL_x],[AVL_y]])

        self.resultant_matrix=np.matmul(self.matrix,self.AVL_matrix)#getting v and omega matrix

        self.drive_command.linear.x=float(self.resultant_matrix[0])#v
        self.drive_command.angular.z=0.2*float(self.resultant_matrix[1])#omega

    def telemetry_publisher(self):
        self.telemetry_publisher_obj = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_rate = rospy.Rate(self.PUB_FREQ)
        print("publishing to /cmd_vel")
        while not rospy.is_shutdown():
            self.drive()
            self.telemetry_publisher_obj.publish(self.drive_command)
            self.pub_rate.sleep()
            

if __name__ == '__main__':
    DriverObj = Driver()
    try:
        DriverObj.telemetry_publisher()
    except rospy.ROSInterruptException():
        pass
