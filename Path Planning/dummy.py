#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
import math 
from math import pi
import numpy as np
from tf.transformations import euler_from_quaternion


LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class PathPlanner:
    def __init__(self):
        rospy.init_node('turtlebot3_path_planner', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.current = Point()
        self.goal = Point()
        self.move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.state = 'neutral'
        
        self.MIN_LIMIT = 0.25

    def obstacle_detector(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        lidar_distances = scan.ranges
        RANGES = scan.ranges
        left_view = list(RANGES[0:91])
        right_view =list(RANGES[270:])

        i=0
        while(i<len(left_view)):
            if(left_view[i]<=scan.range_min):
                self.left_view[i]=10000
            i+=1
        i=0
        while(i<len(right_view)):
            if(right_view[i]<=scan.range_min):
                right_view[i]=10000
            i+=1
            
        min_range_left=min(left_view)
        min_range_right=min(right_view)
        if (min_range_left < self.MIN_LIMIT) or (min_range_right < self.MIN_LIMIT):
            return True
        return False


    def deal_obstacle(self):
        #print('Rounding the obstacle')
        scan = rospy.wait_for_message('scan', LaserScan)
        lidar_distances = scan.ranges
        RANGES = scan.ranges
        left_view = list(RANGES[0:91])
        left_most=list(RANGES[88:93])
        right_view =list(RANGES[270:])
        right_most =list(RANGES[268:273])

        i=0
        while(i<len(left_view)):
            if(left_view[i]<=scan.range_min):
                self.left_view[i]=10000
            i+=1
        i=0
        while(i<len(right_view)):
            if(right_view[i]<=scan.range_min):
                right_view[i]=10000
            i+=1
            
        while(i<len(left_most)):
            if(left_most[i]<=scan.range_min):
                left_most[i]=10000
            i+=1
        i=0
        while(i<len(right_most)):
            if(right_most[i]<=scan.range_min):
                right_most[i]=10000
            i+=1
            
        min_range_left=min(left_view)
        min_left_ind = left_view.index(min_range_left)
        turn_left_ang = 270 - min_left_ind
        turn_left_rad = (turn_left_ang/180)*3.14
        
        
        
        min_range_right=min(right_view)
        min_right_ind = right_view.index(min_range_right)
        turn_right_ang = 90 - min_right_ind
        turn_right_rad = (turn_right_ang/180)*3.14
        
        left_most = min(left_most)
        right_most = min(right_most)
        
        while min(min_range_left,min_range_right) < self.MIN_LIMIT:
            if min_range_left < self.MIN_LIMIT and min_range_right > self.MIN_LIMIT:
                print("Turning right")
                self.move_cmd.angular.z = turn_left_rad
            elif min_range_left > self.MIN_LIMIT and min_range_right < self.MIN_LIMIT:
                print("Turning Left")
                self.move_cmd.angular.z = turn_right_rad
            elif (min_range_left == min_range_right) and  min_range_right < self.MIN_LIMIT:
                print('Turning right')
                self.move_cmd.angular.z = 5
            else:
                pass
            self.cmd_vel.publish(self.move_cmd)
        print('Obstacle Overcame')
        self.Controller()
            
                   
    def goToPoint(self):
        #print('goToPoint')
        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        goalX = self.goal.x
        goalY = self.goal.y
        currentX = self.current.x
        currentY = self.current.y
        dist = np.sqrt((goalY - currentY)**2 + (goalX - currentX)**2)
        path_angle = np.arctan((goalY-currentY)/(goalX-currentX))
        
            
        if path_angle < -pi/4 or path_angle > pi/4:
            if goalY < 0 and currentY < goalY:
                path_angle = -2*pi + path_angle
            elif goalY >= 0 and currentY > goalY:
                path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            self.move_cmd.angular.z = angular_speed * path_angle-rotation
            self.move_cmd.linear.x = min(linear_speed * dist, 0.1)

            #if self.move_cmd.angular.z > 0:
                #self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
            #else:
                #self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(self.move_cmd)
            
            
        
    def getkey(self):
        x, y, z = input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z
    
    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        
    def get_odom(self):
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
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), rotation[2])

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel.publish(stop_msg)
        
    def goal_def(self):
        print('goal_def')
        (self.goal.x, self.goal.y, self.goal.z) = self.getkey()
    def Controller(self):
        #print('controller')
        if self.state == 'neutral':
            self.goToPoint()
            if self.obstacle_detector():
                self.state = 'obs'
            else:
                pass
        elif self.state == 'obs':
            self.deal_obstacle()
        else:
            pass
            
        
   
        #try:
            
            #if self.obstacle_detector():
                #print('Obstacle Detected')
             #   self.stop_robot()
              #  self.cmd_vel.publish(Twist())
             #   self.deal_obstacle()
             #   return
           # else:
                
        #rospy.loginfo('Linear Vel',self.move_cmd.linear.x)
        #except KeyboardInterrupt:
            #rospy.loginfo("Keyboard interrupt received, stopping the program")
            
            
'----------------------------------------------------------'


if __name__ == '__main__':
    Planner = PathPlanner()
    Planner.goal_def()
    try:
        while not rospy.is_shutdown():
            Planner.Controller()
    except Exception as e:
        rospy.logerr('error: {}'.format(e))
        import traceback
        traceback.print_exc()
