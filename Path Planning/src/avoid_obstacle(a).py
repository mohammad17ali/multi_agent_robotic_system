#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math 
import numpy

# Commanded velocity 
move = Twist() # defining the variable to hold values
move.linear.x = 0
move.linear.y = 0
move.linear.z = 0
move.angular.x = 0
move.angular.y = 0
move.angular.z = 0

def controller(v, omega):
    move.linear.x = v
    move.angular.z = omega
    
#############################Edit only this function######
def lidar_callback(msg):
    #Make sense of the callback data
    #rostopic echo msg
    #0 to 359 aray index values with each value at a degree increment 
    #range should be in meters
    #The readings start from left and go counter clockwise
    RANGES=msg.ranges
    #Get range measurements from the front left side of the robot [from 0degrees to 90degrees]
    left=list(RANGES[0:46])
    left_most = list(RANGES[88:93])
    

    #Get range measurements from the front right side of the robot [from 270degrees to 359degrees]
    right=list(RANGES[315:])
    right_most=list(RANGES[268:273])

    
    #Replace zero readings with some large number
    i=0
    while(i<len(left)):
        if(left[i]<=msg.range_min):
            left[i]=10000
        i+=1
        
    i=0
    while(i<len(right)):
        if(right[i]<=msg.range_min):
            right[i]=10000
        i+=1

    
    #Find the minimum range measurement from both sides
    min_range_left = min(left)
    left_ind = left.index(min_range_left)
    min_range_left_rad = (min_range_left*3.14/360)
    left_most_dist = min(list(RANGES[88:93]))

    min_range_right = min(right)
    right_ind = right.index(min_range_right)
    min_range_right_rad = (min_range_right*3.14/360)
    right_most_dist = min(list(RANGES[268:273]))

    if self.obstacle_detected():
	print('Obstacle Detected")
	
	if min_range_left < 0.25 and min_range_right > 0.25:
	    self.stop_bot()
	    print("Turning right")
	    move.angular(-0.225)
	    
	    controller(0.05, -0.225)
        return
	    
        
	

	    
	if min_range_left > 0.4 and min_range_right > 0.4: #Obstacle on left
        print("Drive straight")
        controller(0.05,0.0)
        return
	
    
        
    if min_range_left > 0.4 and min_range_right < 0.4:
        print("Turn left")
        controller(0.05, 0.225)
        return
        
    if min_range_left < 0.2 and min_range_right < 0.2:
        print("Stop robot")
        controller(0.0, 0.0)
        return 
	

    
def wall_follower(
#############################Edit only this function######
  
rospy.init_node('Go_to_goal')  # Defines a node with name of Go_to_goal
velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
lidar_subscriber = rospy.Subscriber('/scan', LaserScan, lidar_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
    velocity_pub.publish(move)
    rate.sleep()

