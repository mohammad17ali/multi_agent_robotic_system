#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
import tf
from math import sqrt, pow, atan2, pi
from sensor_msgs.msg import LaserScan

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class ObstacleAvoider():
    def __init__(self):
        rospy.init_node('obstacle_avoider', anonymous=True)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan_msg):
        lidar_distances = scan_msg.ranges
        min_distance = min(lidar_distances)
        if min_distance < SAFE_STOP_DISTANCE:
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel.publish(stop_msg)

    def move_to_point(self, goal_x, goal_y, goal_z):
        position = Point()
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        goal_z = self.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x - x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle

            move_cmd = Twist()
            move_cmd.angular.z = self.angular_speed(path_angle - rotation[2])
            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(LINEAR_VEL * distance, 0.1)
            self.cmd_vel.publish(move_cmd)

            if self.check_obstacle():
                rospy.loginfo("Obstacle detected. Stopping the robot...")
                self.stop_robot()
                return

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (position, rotation)

    def check_obstacle(self):
        min_distance = min(lidar_distances)
        if min_distance < SAFE_STOP_DISTANCE:
            return True
        return False

    def deg2rad(self, degree):
        return degree * pi / 180

    def angular_speed(self, angle):
        return angle

if __name__ == '__main__':
    try:
        obstacle_avoider = ObstacleAvoider()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
