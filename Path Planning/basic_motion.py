import cv2
import cv2.aruco as aruco
import pyrealsense2
import matplotlib.pyplot as plt
import numpy as np
from realsense_depth import *
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

###Functions###

def coordinates(calib_coords,x_,y_,theta):
    x1,y1,x2,y2 = calib_coords
    tan = np.tan(theta)
    cos = np.cos(theta)
    sec = 1/(np.cos(theta))
    cosec = 1/(np.sin(theta))
    y_tmp = (y_ - y1 - (x_- x1)*tan)*sec
    x_tmp = (x_ - x1)*sec + y_tmp*tan
    s_ = np.sqrt((y2-y1)**2 + (x2-x1)**2)
    dist_fac = s_/s
    x = round((x_tmp/dist_fac),2)
    y = round((y_tmp/dist_fac),2)
    
    
    return x,y
def angle(y1_,y2_,x1_,x2_):
    tmp = float(y2_ - y1_)/float(x2_ - x1_)
    invtan = np.arctan(tmp)
    return invtan

## Getting coordiinates##

dc = DepthCamera()

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)



try:               
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        
        corners, ids, _ = cv2.aruco.detectMarkers(color_image, aruco_dict, parameters=aruco_params)

        calib_coords = [] 
        if ids is not None:
            for i in range(len(ids)):
                if ids[i] == 1:
                    a_corner = corners[i]
                    p1 = np.mean(a_corner[0], axis=0).astype(int)
                    depth_value = depth_frame.get_distance(p1[0], p1[1])
                    if depth_value > 0:
                        depth_point = rs.rs2_deproject_pixel_to_point(
                            depth_frame.profile.as_video_stream_profile().intrinsics, [p1[0], p1[1]], depth_value)
                        x1__ = depth_point[0]
                        y1__ = depth_point[1]
                        z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                        org_a = (int(a_corner[0, 0, 0]), int(a_corner[0, 0, 1]) - 10)
                if ids[i] == 2:
                    b_corner = corners[i]
                    p2 = np.mean(b_corner[0], axis=0).astype(int)
                    depth_value = depth_frame.get_distance(p2[0], p2[1])
                    if depth_value > 0:
                        depth_point = rs.rs2_deproject_pixel_to_point(
                            depth_frame.profile.as_video_stream_profile().intrinsics, [p2[0], p2[1]], depth_value)
                        x2__ = depth_point[0]
                        y2__ = depth_point[1]
                        z2 = depth_point[2] # no need for separate z1_, as its value won't be changed
                        org_b = (int(b_corner[0, 0, 0]), int(b_corner[0, 0, 1]) - 10) 

                x1_, y1_, x2_,y2_ = round(x1__,2),round(y1__,2), round(x2__,2), round(y2__,2)
                calib_coords = [x1_, y1_, x2_,y2_]
                theta = angle(y1_,y2_,x1_,x2_)

                if (1 in ids) and (2 in ids):
                    x1,y1 = coordinates(calib_coords,x1_,y1_,theta)
                    x2,y2 = coordinates(calib_coords,x2_,y2_,theta)
                    marker_text_a = "A({:.2f}, {:.2f})".format( x1, y1)
                    marker_text_b = "B({:.2f}, {:.2f})".format(x2, y2)
                    cv2.line(color_image, p1, p2,(255,0,0), 1)
                    cv2.putText(color_image, marker_text_a, org_a, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(color_image, marker_text_b, org_b, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                else:
                    break

        if ids is not None:
            for i in range(len(ids)):
                if ids[i] == 4: #id16 = starting point tag
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                    center = np.mean(corners[i][0], axis=0).astype(int)
                    depth_value = depth_frame.get_distance(center[0], center[1])
                    if depth_value > 0:
                        depth_point = rs.rs2_deproject_pixel_to_point(
                            depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
                        start_x_ = depth_point[0]
                        start_y_ = depth_point[1]
                        start_z = depth_point[2] # no need for separate z1_, as its value won't be changed
                        start_org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
                        
                        start_x, start_y = coordinates(calib_coords, start_x_,start_y_, theta)
                        marker_text_start = "P1({:.2f}, {:.2f})".format( start_x, start_y)
                        cv2.putText(color_image, marker_text_start, start_org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                elif ids[i] == 3: #id30 = target point tag
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                    center = np.mean(corners[i][0], axis=0).astype(int)
                    depth_value = depth_frame.get_distance(center[0], center[1])

                    if depth_value > 0:
                        depth_point = rs.rs2_deproject_pixel_to_point(
                            depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
                        target_x_ = depth_point[0]
                        target_y_ = depth_point[1]
                        target_x = depth_point[2] # no need for separate z1_, as its value won't be changed
                        target_org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
                        
                        target_x, target_y = coordinates(calib_coords, target_x_,target_y_, theta)
                        marker_text_target = "P2({:.2f}, {:.2f})".format( target_x, target_y)
                        cv2.putText(color_image, marker_text_target, target_org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                
        cv2.imshow("ArUco Marker Detection", color_image)

        # Exit the program when the 'Esc' key is pressed
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

#plt.figure(figsize = (10,10))
#plt.imshow(color_image)
start = [start_x, start_y]
target = [target_x, target_y]
print(f'Starting point is ({start_x},{start_y})')
print(f'T point is ({target_x},{target_y})')

### Motion of Bots##


class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.target_reached = False
        self.target_tolerance = 0.1  # Tolerance for considering the target reached
        self.linear_speed = 0.2  # Linear speed of the robot
        self.angular_speed = 0.5  # Angular speed of the robot

    def odom_callback(self, data):
        # Extract robot's position and orientation from odometry data
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.robot_x = data.pose.pose.position.x
        self.robot_y = data.pose.pose.position.y
        self.robot_yaw = yaw

    def move_forward(self, distance):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        current_x = self.robot_x
        current_y = self.robot_y
        distance_moved = 0

        while distance_moved < distance:
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)  # Wait for a short duration
            distance_moved = math.sqrt((self.robot_x - current_x)**2 + (self.robot_y - current_y)**2)

        twist_msg.linear.x = 0  # Stop moving forward
        self.cmd_vel_pub.publish(twist_msg)

    def rotate_to_orientation(self, target_orientation):
        twist_msg = Twist()
        angular_tolerance = 0.05  # Tolerance for considering the target orientation reached

        while not rospy.is_shutdown():
            angle_difference = target_orientation - self.robot_yaw

            if abs(angle_difference) < angular_tolerance:
                break  # Target orientation reached

            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            twist_msg.angular.z = self.angular_speed if angle_difference > 0 else -self.angular_speed
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)  # Wait for a short duration

        twist_msg.angular.z = 0  # Stop rotating
        self.cmd_vel_pub.publish(twist_msg)

    def calculate_angle_to_target(self, target_x, target_y):
        angle_to_target = math.atan2(target_y - self.robot_y, target_x - self.robot_x)
        return angle_to_target

    def navigate_to_target(self, target_x, target_y):
        angle_to_target = self.calculate_angle_to_target(target_x, target_y)
        self.rotate_to_orientation(angle_to_target)

        distance_to_target = math.sqrt((target_x - self.robot_x)**2 + (target_y - self.robot_y)**2)
        self.move_forward(distance_to_target)

        # Check if the target is reached
        if distance_to_target < self.target_tolerance:
            self.target_reached = True

if __name__ == '__main__':
    try:
        controller = RobotController()
        # Example usage:
        start_x, start_y = start
        target_x, target_y = target
        controller.navigate_to_target(target_x, target_y)
        if controller.target_reached:
            rospy.loginfo("Target reached!")
        else:
            rospy.loginfo("Failed to reach the target.")
    except rospy.ROSInterruptException:
        pass
