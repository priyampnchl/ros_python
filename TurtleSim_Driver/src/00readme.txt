Assignment 2 ELG 5228 Readme

Dependencies: 
rospy
turtlesim

Message types used:
turtlesim_msgs.msg/Pose
geometry_msgs.msg/Twist

Steps to run:

Create a package in the source (src) directory of your catkin workspace (here it is assumed to be ~/catkin_ws and it is also assumed that your terminal is opened in ~/catkin_ws/src) with "catkin_create_pkg <package_name> rospy turtlesim". This will create a directory with the name <package_name> and create a directory within it with the name "src". 

The files target_publisher.py, pose_reporter.py and robot_driver.py need to be placed in the source (src) folder of the package directory (~/catkin_ws/). and made executable.

Build your package from ~/catkin_ws/ using "catkin build <package_name>", then source it using "source ~/catkin_ws/devel/setup.bash 

Verify that it is created using "rospack find <package_name>", if not, refer to the slides on BrightSpace :) 

Running the nodes:

1. Run roscore using the command “roscore”
2. Run turtlesim using the command “rosrun turtlesim turtlesim_node” 
3. Run target_publisher.py using the command “rosrun <package_name> target_publisher.py”
3.A. Enter the target X coordinate as per the prompt
3.B. Enter the target Y coordinate as per the prompt
3.C. If both are entered correctly the node will publish the target coordinates to the topic “/turtle1/destination” and log the same on the console in the format: [INFO] [1668449390.960005]: X: 3.0 metres,Y: 5.0 metres
4. Run pose_reporter.py using the command “rosrun <package_name> pose_reporter.py”. The node will subscribe to the topic /turtle1/destination and /turtle1/pose and publish the current X-coordinate, current Y-coordinate, distance from target, current orientation and orientation error to the topic “/turtle1/driverinfo”. It also logs the same on the console as follows: [INFO] [1668527631.944532]: X:5.544 metres, Y:5.544 metres, Distance:3.485 metres, Current Orientation:0.000 degrees, Orientation Error:82.490 degrees
5. Run robot_driver.py using the command “rosrun <package_name> robot_driver.py”. The node will subscribe to the topic “/turtle1/driverinfo” and compute the required command velocities and publish them to the topic “/turtle1/cmd_vel” until the turtle gets to within 10cm of the goal destination. It will log on the console the current state of the turtle which is “NAVIGATING” when moving towards the destination or “DESTINATION REACHED” once it reaches (within 10cm of) the destination. 

Now I will explain the functionality of each node. 

2A: target_publisher.py

This node prompts the user for coordinates and publishes them over the topic /turtle1/destination at a rate of 10Hz (verifiable using rostopic hz target_publisher.py). 
Once target_publisher.py is run using rosrun, it prints the message "node target_publisher created". This means that the node is active and will show up on rqt_graph.
A prompt is printed on the next line to enter coordinate. After the prompt, enter a numeric value following the conditions in the prompt and press enter. This will be the first (or X) co-ordinate. Repeat for the Y coordinate. Note that it will keep prompting you for the coordinates until you enter the proper value, i.e. a value that is a float between 0 and 11 (including both). The node will be initialised as soon as you run it using rosrun (verifiable using rqt_graph), but nothing will be published to the topic /turtle1/destination until BOTH the target coordinates (x and Y) are entered correctly.
The node continuously publishes the target coordinates to /turtle1/destination and the same coordinates are logged on the terminal using the rospy.loginfo method.

2B: pose_reporter.py
This node listens to (i.e. is subscribed to) the topics /turtle1/pose and /turtle1/destination (verifiable using rqt_graph) and publishes to the topic /turtle1/driverinfo at 2Hz. It takes the current pose and target coordinates and computes the following:

1. The euclidean distance between the current coordinates (as read from /turtle1/pose) and the target coordinates (as read from /turtle1/destination)
2. The orientation error between the current orientation (as read from /turtle1/pose) and target orientation, which is computed using the math.atan2() function
3. The forward velocity to be applied to the turtle, proportional to the distance between the turtle and the target point (as described in point 1).
4. The angular velocity to be applied to the turtle proportional to the orientation error between the current and target orientation (as described in point 2).

The node logs the current X and Y coordinates, the distance to target, the current orientation and the target orientation using rospy.loginfo at a rate of 2Hz. 

The same signals are also published to the topic /turtle1/driverinfo.

2C: robot_driver.py
This node subscribes to the /turtle1/driverinfo topic to receive the relevant error signals. The node then uses the distance error and the orientation error to generate proportional control signals for the linear and angular velocities of the turtle. It turns left or right based on whether the target is to its left or right for minimum possible rotation. It mostly is able to converge to the target in a reasonable time.

Edge cases: If the orientation error is over roughly 160 degrees in either direction, it will not converge and start oscillating. This is because the coefficients are larger to have it be able to reach most of the canvas within a reasonable amount of time. I know that an easier implementation would be to first turn until angle error is zero then walk straight, while correcting the angle error that accumulates during the straight moves, but this way is more elegant (in my opinion) and can be improved upon using an integral term. The time taken to reach the target is majorly constrained by the low publishing rate of pose_reporter.py. My 2nd submission performs better because I mistakenly used a publishing frequency of 10Hz which allowed me to have larger coefficients for the linear and angular velocities which allowed the turtle to turn faster without overshooting. With the constraints of the assignment however it is difficult to get the turtle to converge sooner, and it has to stop and turn at times probably because of the 10x variation in the publishing rate of 2B and 2C. For most points it will work fine, but performance drops when the angle error is higher in proportion to distance error. I have had to make the turtle super slow (because it’s a turtle haha) but it would converge way faster if the pose_reporter pub rate was quicker. It still is able to get to its destination fairly quickly (under a minute on my system)