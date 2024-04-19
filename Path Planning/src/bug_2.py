#! /usr/bin/env python


import rospy

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion
from goal_publisher.msg import PointArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *

import math

point_x=[0, 0, 0]
point_y=[0, 0, 0]
point_z=[0, 0, 0]
def callback(msg):
	global point_x
	global point_y
	global point_z
	for i in range (len(msg.goals)):
		point_x[i]=(msg.goals[i].x)
		point_y[i]=(msg.goals[i].y)
		point_z[i]=(msg.goals[i].z)


file_point_follow = None
file_wall_follow = None

theta_tolerance = 5 * (math.pi / 180)
area = None
state = 0
wall_follow_time = 0
loop_number = 0

init_position=Point()
init_position.x=0
init_position.y=0
init_position.z=0

dest_position=Point()
dest_position.x=0
dest_position.y=0
dest_position.z=0
vel=Twist()
i=0
x=y=z=theta=0
def clbk_odom(msg):
    global x
    global y
    global theta
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation
    (roll,pitch,theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

def clbk_laser(msg):
    global area
    area = {
        'front' : min(min(msg.ranges[0:35]),min(msg.ranges[325:359]),3.5),
		'fleft' : min(min(msg.ranges[36:107]),3.5),
		'left' : min(min(msg.ranges[108:179]),3.5),
		'right' : min(min(msg.ranges[180:251]),3.5),
		'fright' : min(min(msg.ranges[252:324]),3.5)
    }

def change_state(value):
    global state
    global file_wall_follow, file_point_follow
    global wall_follow_time
    wall_follow_time = 0
    state = value
    if state == 0:
        resp = file_point_follow(True)
        resp = file_wall_follow(False)
    if state == 1:
        resp = file_point_follow(False)
        resp = file_wall_follow(True)


def main():
    global area, dest_position, state, theta, theta_tolerance, x, y,i
    global file_point_follow, file_wall_follow
    global wall_follow_time, loop_number, init_position, dest_position,point_x,point_y,point_z
    rospy.init_node('bug2')

    pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_goal = rospy.Subscriber("/goals", PointArray, callback)
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')

    file_point_follow = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    file_wall_follow = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    point_x=[3.5,7,-6,5]
    point_y=[9,-9.5,-13.5]
    dest_position=Point()
    dest_position.x=point_x[0]
    dest_position.y=point_y[0]
    dest_position.z=0
    print(dest_position)
    change_state(0)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if area == None:
            continue


	num = math.fabs((dest_position.y - init_position.y) * x - (dest_position.x - init_position.x) * y + (dest_position.x * init_position.y) - (dest_position.y * init_position.x))
        den = math.sqrt(pow(dest_position.y - init_position.y, 2) + pow(dest_position.x - init_position.x, 2))
        per_distance = num / den
	print(per_distance)
	print(den)


	if math.sqrt(pow(dest_position.y - y, 2) + pow(dest_position.x - x, 2)) <= 0.3:

					init_position.x=point_x[i]
					init_position.y=point_y[i]
					init_position.z=0
					i=i+1
					dest_position.x=point_x[i]
					dest_position.y=point_y[i]
					dest_position.z=0
					print(dest_position)
					change_state(0)


        if state == 0:
			if area['front'] > 0.15 and area['front'] < 0.4:
				vel.linear.x=0
				vel.angular.z=0
				pub.publish(vel)
				change_state(1)



        elif state == 1:
            if wall_follow_time > 13 and per_distance < 0.15:
				vel.linear.x=0
				vel.angular.z=0
				pub.publish(vel)
		                change_state(0)



	if i>2:
				continue

        loop_number = loop_number + 1
        if loop_number == 20:
            wall_follow_time = wall_follow_time + 1
            loop_number = 0

        rate.sleep()

if __name__ == "__main__":
    main()
