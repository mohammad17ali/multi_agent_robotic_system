#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion
from goal_publisher.msg import PointArray
from std_srvs.srv import *

import math

active = False

# machine state
state = 0

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
#	print(point_x)

init_position=Point()
init_position.x=0
init_position.y=0
init_position.z=0

dest_position=Point()
dest_position.x=0
dest_position.y=0
dest_position.z=0


# parameters
theta_tolerance = 2*math.pi / 90
dist_tolerance = 0.3
i=0
# publishers
pub = None

# service callbacksmsg.goals[i].x
def go_to_point_switch(req):
    global active
    active = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks
x=y=z=theta=0
def clbk_odom(msg):
    global x
    global y
    global theta
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation
    (roll,pitch,theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

def change_state(value):
    global state
    state = value

def calculate_theta(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def set_angle(dest_position):
    global theta, pub, theta_tolerance, state
    desired_theta = math.atan2(dest_position.y - y, dest_position.x - x)
    error_theta = calculate_theta(desired_theta - theta)

#	twist_msg = Twist()

    if math.fabs(error_theta) > theta_tolerance:

	twist_msg = Twist()
        twist_msg.angular.z = 0.4 if error_theta > 0 else -0.4
	pub.publish(twist_msg)

    # state change conditions
    if math.fabs(error_theta) <= theta_tolerance:
    	change_state(1)


def go_point(dest_position):
    global theta, pub, theta_tolerance, state
    desired_theta = math.atan2(dest_position.y - y, dest_position.x - x)
    error_theta = desired_theta - theta
    error_position = math.sqrt(pow(dest_position.y - y, 2) + pow(dest_position.x - x, 2))

    if error_position > dist_tolerance:
        twist_msg = Twist()
        twist_msg.linear.x = 0.7

        pub.publish(twist_msg)
    else:
        change_state(2)


    if math.fabs(error_theta) > theta_tolerance:
        change_state(0)


def main():
    global pub, active,i
    global point_x, point_y, point_z
    rospy.init_node('go_to_point')
    pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_goal = rospy.Subscriber("/goals", PointArray, callback)

    point_x=[3.5,7,-6,5]
    point_y=[9,-9.5,-13.5]
    dest_position=Point()
    dest_position.x=point_x[i]
    dest_position.y=point_y[i]
    dest_position.z=0
    print(dest_position)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active:
            continue
        else:
	    if state == 0:
                set_angle(dest_position)
            elif state == 1:
                go_point(dest_position)
            elif state == 2:
		            twist_msg = Twist()
			    twist_msg.linear.x = 0
			    twist_msg.angular.z = 0
			    pub.publish(twist_msg)
			    init_position.x=x
			    init_position.y=y
			    init_position.z=0
			    i=i+1
			    dest_position.x=point_x[i]
			    dest_position.y=point_y[i]
			    dest_position.z=0
			    change_state(0)
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()

if __name__ == '__main__':
    main()
