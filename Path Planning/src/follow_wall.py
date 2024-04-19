#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active = False
pub = None
area = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state = 0

def wall_follower_switch(req):
    global active
    active = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_laser(msg):
    global area
    area = {
        'front' : min(min(msg.ranges[0:35]),min(msg.ranges[325:359]),3.5),
		'fleft' : min(min(msg.ranges[36:107]),3.5),
		'left' : min(min(msg.ranges[108:179]),3.5),
		'right' : min(min(msg.ranges[180:251]),3.5),
		'fright' : min(min(msg.ranges[252:324]),3.5)
    }

    take_action()

def change_state(value):
    global state
    state = value

def take_action():
    global area
    msg = Twist()
    linear_x = 0
    angular_z = 0

    d = 0.4

    if area['front'] > d and area['fleft'] > d and area['fright'] > d:
        change_state(0)
    elif area['front'] < d and area['fleft'] > d and area['fright'] > d:
        change_state(1)
    elif area['front'] > d and area['fleft'] > d and area['fright'] < d:
        change_state(2)
    elif area['front'] > d and area['fleft'] < d and area['fright'] > d:
        change_state(0)
    elif area['front'] < d and area['fleft'] > d and area['fright'] < d:
        change_state(1)
    elif area['front'] < d and area['fleft'] < d and area['fright'] > d:
        change_state(1)
    elif area['front'] < d and area['fleft'] < d and area['fright'] < d:
        change_state(1)
    elif area['front'] > d and area['fleft'] < d and area['fright'] < d:
        change_state(0)
    else:
        rospy.loginfo(area)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.4
    msg.angular.z = -0.2
    return msg

def turn_left_angle():
    msg = Twist()
    msg.linear.x=0.0
    msg.angular.z = 0.4
    return msg


def follow_wall():
    global area

    msg = Twist()
    msg.linear.x = 0.4
    msg.angular.z=0
    return msg



def main():
    global pub, active, area

    rospy.init_node('reading_laser')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active:
            continue
        print("hi")
        msg=Twist()
        if state==0:
            msg=find_wall()
        elif state==1:
            msg=turn_left_angle()
        elif state==2:
            msg=follow_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
