#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#global variables for the target destinations, I did not end up using yaw because of the janky spot-turns in husky, but it was interesting nonetheless 
xtarg=0
ytarg=0
yaw=0

class DriverBhaya():

    PUB_FREQ=6#Hz - Publishing rate 
    TOLERANCE=0.35#metres - Allowed delta 
    CCMVENT_DIST=0.7#metres - Circumvent distance
    LINEAR_VEL=0.15#m/s - default linear velocity
    ANGULAR_VEL=0.2#rad/s - default angular velocity


    def __init__(self):
        self.telemetry=Pose2D() #initialise telemetry object of type Pose2D
        self.odometry=Odometry() #initialise odometry object of type Odometry
        self.drive_command=Twist() # #initialise drive_command object of type Twist
    
        self.obstacle_found=False #set if obstacle is visible
        self.obstacle_reached=False # set if husky is within self.CCMVENT_DIST of obstacle
        self.circumventing=False # set if circumventing
        self.reached_initial_targ=False# set if positiiion where you started circumventing is reached
        self.finished=False #unused, idea was to set when all done, unused to be able to keep deleting obstacles to reset everything 
        self.check_target=False #set if we need to check distance_to_target

        self.sub=rospy.Subscriber('/sensed_object', Pose2D, self.parse_telemetry) #initialise subscriber and subscribe to /sensed_object

        self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.compute_odom) #initialise subscriber and subscribe to /odometry/filtered

        rospy.init_node('husky_driver', anonymous=False)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)#initialise the publisher
        self.pub_rate = rospy.Rate(self.PUB_FREQ)
        print("publishing to /cmd_vel")

    def compute_odom(self,msg):#function to compute the current x and y coorfinates and the yaw from the quaternion we get from odometry.pose.pose.orientation
        self.odometry=msg
        self.xcoord=self.odometry.pose.pose.position.x
        self.ycoord=self.odometry.pose.pose.position.y
        (self.roll,self.pitch,self.yaw)=euler_from_quaternion([self.odometry.pose.pose.orientation.x,self.odometry.pose.pose.orientation.y,self.odometry.pose.pose.orientation.z,self.odometry.pose.pose.orientation.w]) #compute RPY
        self.yaw=self.yaw+math.pi

    def parse_telemetry(self, msg):#callback function
        self.telemetry=msg

    def get_current_pose(self, msg):#callback function
        self.current_pose=msg

    def detect_obstacle(self):
        if (math.isnan(self.telemetry.x) and math.isnan(self.telemetry.theta)): #check if lidar is sending nan
            #reset all flags if no obstacle so I don't have to keep Ctrl+C'ing 
            self.obstacle_found=False 
            self.obstacle_reached=False 
            self.circumventing=False 
            self.reached_initial_targ=False
            self.finished=False
            self.check_target=False 

            global xtarg
            global ytarg
            global yaw

            #reset global vars that may get wrongly populated because the first few messages from odometry are zero 
            xtarg=0
            ytarg=0
            yaw=0

            self.stop()#stop if no obstacles, duh
            
            print("NO OBSTACLE FOUND: NaN DETECTED")

            # print(self.yaw)
        else: #if lidar data is non-NaN
            self.obstacle_found=True
            # print("OBSTACLE FOUND") 
   
    def turn_left(self,v1=0,v2=ANGULAR_VEL):#it's in the name
        print("TURN LT")
        self.drive_command.linear.x = v1
        self.drive_command.angular.z = v2

    def turn_right(self,v1=0,v2=ANGULAR_VEL):#it's in the name
        print("TURN RT")
        self.drive_command.linear.x = v1
        self.drive_command.angular.z = -v2

    def move_fwd(self,v1=LINEAR_VEL):#it's in the name
        print("FORWARD")
        self.drive_command.linear.x = v1
        self.drive_command.angular.z = 0

    def move_back(self, v1=LINEAR_VEL):#it's in the name
        print("BACKWARD")
        self.drive_command.linear.x = -self.LINEAR_VEL/2
        self.turn_right()

    def stop(self):#it's in the name
        self.drive_command.linear.x=0
        self.drive_command.angular.z=0

    
    def gotogoal(self, dist = CCMVENT_DIST):#it's in the name
        # print("X and ccmv_dist+tolerance", self.telemetry.x, dist + self.TOLERANCE)
        if self.telemetry.x >= dist: 
            print("GOING TO TARGET")
            #turn fast for higher values of bearing, slow down to fine-tune because husky is jank when it comes to turning
            if self.telemetry.theta > 30:
                self.turn_right(0,0.3)
            elif self.telemetry.theta > 3:
                self.turn_right(0)
            elif self.telemetry.theta < -30:
                self.turn_left(0,0.3) 
            elif self.telemetry.theta < -3:
                self.turn_left(0)
            else:
                self.move_fwd()
        elif self.telemetry.x < dist+0.1 and not self.obstacle_reached:#record point when distance became <= 0.7m from obstacle 
            self.obstacle_reached=True
            self.stop()
            global xtarg
            xtarg=self.xcoord
            print("recorded coordinate x")
            global ytarg 
            ytarg=self.ycoord
            print("recorded coordinate y")
            global yaw
            yaw=self.yaw
            print("recorded yaw")
        else:
            self.stop()
        
    def check_dist(self):#function to only check distance from target 
        # print("check_target flag", self.check_target)
        if self.distance_to_target > 1:
            self.check_target=True

    def circumvent(self):#function to circumvent, it took every ounce of my soul to make this (sort of) work :')
        self.x_error=self.telemetry.x-self.CCMVENT_DIST
        self.angle_error=abs(self.telemetry.theta-90)
        print("AND CIRCUMVENTING")
        # print("x error",xtarg-self.xcoord,"y error",ytarg-self.ycoord,"yaw err",yaw-self.yaw+1.57)
        self.check_dist()

        if self.check_target and self.distance_to_target<1:  #change this value if it keeps circumventing || bop out of circumvent once initial point is reached, i.e., one turn is completed
            self.reached_initial_targ=True
            return

        if not self.reached_initial_targ:#this bit makes the husky circumvent
            print("entered main if")   
            
            if self.CCMVENT_DIST-self.TOLERANCE/2 < self.telemetry.x < self.CCMVENT_DIST+self.TOLERANCE/2:
                # print("entered circumvent main if")                
                if -135 < self.telemetry.theta < 80:#turn until closest point on object to your right basically
                    # print("circumvent main if case 1")
                    self.turn_left(0.07)
                elif 80 <= self.telemetry.theta < 95:#move forward but dont turn while object to your right
                    # print("circumvent main if case 2")
                    self.move_fwd()
                elif 95 <= self.telemetry.theta < 135:#turn so that closest point on object is to your right ,same as before
                    # print("circumvent main if case 3")
                    self.turn_right(0.07)

            elif self.telemetry.x > self.CCMVENT_DIST+ self.TOLERANCE/2:#turn inward if you stray too far away
                # print("entered circumvent main elif 1")
                self.turn_right(-0.02,0.2)

            elif self.telemetry.x < self.CCMVENT_DIST-self.TOLERANCE/2:#nudge outwards if you get too close
                # print("entered circumvent main elif 2")
                self.turn_left(-0.02,0.2)

            else:#not sure why this fixed things, but hey, I will not  question it. 
                self.move_fwd(0.1)

        else:#same as previous, control will never get passed here but just in case? 
            self.drive_command.angular.z=0               

    def drive_husky(self):#This bit calls everything else and gets things moving
        
        self.distance_to_target=math.sqrt((xtarg-self.xcoord)**2 + (ytarg-self.ycoord)**2)#robot displacement from point when it first got to within 0.7m of obstacle

        # print("distance to target",self.distance_to_target)
        
        self.detect_obstacle() #pretty self explanatory       
        
        if self.obstacle_found:#if oyou can see an obstacle 
            # diff=self.telemetry.x-self.CCMVENT_DIST
            if not self.obstacle_reached:
                self.gotogoal()#do this until you're within 0.7m of the obstacle 
            # elif self.obstacle_reached:
            #      print("recorded coordinates are", xtarg,ytarg)
            elif not self.reached_initial_targ:#keep  circumventing until you reach the initial point when you first got to within 0.7m of the obstacle 
                self.check_dist()
                self.circumvent()
            else:
                self.gotogoal(0.35)#go to the obstacle more until distance becimes 0.35


    def publisherFunction(self):
            while not rospy.is_shutdown():
                # print("obstacle found:",self.obstacle_found)
                # print("obstacle reached:",self.obstacle_reached)
                # print("circumventing:",self.circumventing)
                # print("finished",self.finished)
                self.drive_husky()
                self.pub.publish(self.drive_command)   #publish 
                self.pub_rate.sleep()

    def comments():
                # elif self.CCMVENT_DIST < self.telemetry.x <= self.CCMVENT_DIST-self.TOLERANCE:
                #     self.cicumvent()

                # elif (you are between 0.7 to 0m from obstacle):
                #     record current position x,y,yaw
                #     turn left until obstacle to your riht (80<theta<90)
                    
                # elif obstacle you are right
                        
                # else:
                #     self.obstacle_reached = True
                #     self.stop()
                #     self.define_target()

                # self.circumventing=True
                # if self.circumventing:
                #     if not self.finished:
                #         #circumvent code here
                # print(self.target_location)
                pass

if __name__=='__main__':
    driverObj=DriverBhaya()    #instantiate a class object, which will run the stuff in __init__ (SCROLL UP)
    while not rospy.is_shutdown():
        try:
            driverObj.publisherFunction()            
        except rospy.ROSInterruptException():
            pass