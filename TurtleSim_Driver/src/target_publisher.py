#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose

class TargetPublisherClass():

    FREQ=10#Hz

    def __init__(self):
        # print("__init__ is working")
        self.target=Pose()  #instantiate Pose() object: contains structure of Pose information
        self.axes=["X","Y"]

        rospy.init_node('target_publisher',anonymous=False)     #initialise the node "target_publisher"
        print("node target_publisher created")

        self.target.x=self.get_coordinate(self.axes[0])  #get X coordinate
        self.target.y=self.get_coordinate(self.axes[1])  #get Y coordinate

        self.pub=rospy.Publisher('/turtle1/destination',Pose,queue_size=10)     #create publisher object self.pub
        self.rate=rospy.Rate(self.FREQ)    #define the rate
        print("publishing to /turtle1/destination")

    def publisherFunction(self):
        while not rospy.is_shutdown():
            # print("publisherFunction is running")   #some comfort
            rospy.loginfo("X: %s metres,Y: %s metres",self.target.x,self.target.y)  #log the same shi that you are publishing
            self.pub.publish(self.target)   #publish target
            self.rate.sleep()

    def get_coordinate(self,a):
        # print("get coordinate is working")
        while True:#Look at this bomb ass input validation, fuk yea, present Priyam is proud of past Priyam
            try:
                # print("try block within get_coordiate is working")
                ip=round(float(input("Enter " + a + "-coordinate in metres (float between 0 and 11): ")),3)
                if 0<=ip<=11:
                    return ip
            except:
                pass

if __name__=='__main__':
    tarPubObj=TargetPublisherClass()    #instantiate a TargetPublisherClass, which will run the stuff in __init__ (SCROLL UP)
    while not rospy.is_shutdown():
        try:
            tarPubObj.publisherFunction()
        except rospy.ROSInterruptException():
            pass
    
