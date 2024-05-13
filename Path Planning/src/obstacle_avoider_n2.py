#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from sensor_msgs.msg import LaserScan
from math import radians, sqrt, pow, atan2, pi
from tf.transformations import euler_from_quaternion
import numpy as np

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""
MIN_WALL_DIST = 0.25

class PathPlanner():
    def __init__(self):
        rospy.init_node('obstacle_avoider', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.phase = 'gtp'  # Initialize phase to go to point
        self.obs = 'null'
        self.wall = 'null'
        self.MIN_WALL_DIST = 0.25
        self.move_cmd = Twist()
        self.goal_x, self.goal_y, self.goal_z = self.get_goal()
        self.lowest_allowed_dist = 0.1
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
        print('init')

    def run(self):
        
        print('run')
        r = rospy.Rate(10)
        if self.obs == 'null':
            self.go_to_point()
            r.sleep()
        else:
            print('else')
            self.deal_obstacle()
            r.sleep()
        self.obstacle_detector()
        
         

    def go_to_point(self):
        print('gtp')
        position, rotation = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        goal_x, goal_y, goal_z = self.goal_x, self.goal_y, self.goal_z 
        
        if goal_z > 180 or goal_z < -180:
            rospy.logerr("Invalid orientation angle. Range should be -180 to 180 degrees.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance
        if distance < 0.1:
            print('Goal Reached')
            self.shutdown()

        while distance > 0.1:
            position, rotation = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x - x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2 * pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2 * pi + path_angle
            if last_rotation > pi - 0.1 and rotation <= 0:
                rotation = 2 * pi + rotation
            elif last_rotation < -pi + 0.1 and rotation > 0:
                rotation = -2 * pi + rotation

            self.move_cmd.angular.z = angular_speed * path_angle - rotation
            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            self.move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if self.move_cmd.angular.z > 0:
                self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
            else:
                self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(self.move_cmd)
            rospy.sleep(0.1)
            if self.obstacle_detector():
                break
            else:
                pass
            

        while abs(rotation - goal_z) > 0.05:
            position, rotation = self.get_odom()
            #move_cmd = Twist()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    self.move_cmd.angular.z = 0.5
                else:
                    self.move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    self.move_cmd.angular.z = -0.5
                else:
                    self.move_cmd.angular.z = 0.5
            self.cmd_vel.publish(self.move_cmd)
            rospy.sleep(0.1)

        #rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def get_goal(self):
        print(msg)
        x, y, z = input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z
      
    def obstacle_detector(self):
        print('obstacle_detector')
        wall_limit = 0.1
        scan = rospy.wait_for_message('scan', LaserScan)
        ranges = scan.ranges
    
        # Extracting front, left, and right views from the lidar scan
        front_view = ranges[0:45] + ranges[315:]
        left_view = ranges[0:45]
        right_view = ranges[315:]
        leftm = ranges[88:93]
        rightm = ranges[268:273]
        leftm_min = min(leftm)
        rightm_min = min(rightm)
        if min(ranges) < self.lowest_allowed_dist:
            print('Obstacle detected too close')
            self.controller(-0.5, 0)
            rospy.sleep(0.5)
    
        # Check for obstacles in the front view
        if rightm_min < wall_limit:
            self.wall = 'right'
            #return 'wall_r'
        else:
            self.wall = 'null'
        if rightm_min < wall_limit:
            self.wall = 'left'
            #return 'wall_l'
        else:
            self.wall = 'null'
            
        
        if min(left_view) < self.MIN_WALL_DIST:
            self.obs = 'left'
            #print('Obstacle on Left')
        if min(right_view) < self.MIN_WALL_DIST:
            #print('Obstacle on Right')
            self.obs = 'right'
    
        # Check for obstacles in the left and right views to detect wall obstacles
        if min(front_view) < self.MIN_WALL_DIST:
            rospy.loginfo("Front obstacle detected")
            return True
        
    
        # If no obstacles are detected
        #self.obs = 'null'
        #self.wall = 'null'
        return False

    def deal_obstacle(self):
        print('deal_obstacle')
        if self.wall == 'null':
            if self.obs == 'left':
                print('Obstacle on Left')
                self.controller(0.2, -1)
            elif self.obs == 'right':
                print('Obstacle on Right')
                self.controller(0.2, 1)
            else:
                pass
        if self.wall == 'left':
            if self.obstacle == 'left':
                print('Following wall on left')
                self.controller(0.5,-0.2)
        if self.wall == 'right':
            if self.obstacle == 'right':
                print('Following wall on right')
                self.controller(0.5,0.2)
          
        #rospy.loginfo("Obstacle cleared, continuing navigation")
 


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans), rotation[2]

    def shutdown(self):
        rospy.loginfo("Shutting down...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        
    def stop_robot(self):
        print('Stopping Robot')
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        
        self.cmd_vel.publish(self.move_cmd)
      
    def controller(self,vel,ang):
        print('controller')
        self.move_cmd.linear.x = vel
        self.move_cmd.angular.z = ang
        
        self.cmd_vel.publish(self.move_cmd)
        self.obstacle_detector()
        
        print('Linear Velocity: ', self.move_cmd.linear.x)
        print('Angular Velocity: ', self.move_cmd.angular.z)

if __name__ == '__main__':
    try:
        planner = PathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
