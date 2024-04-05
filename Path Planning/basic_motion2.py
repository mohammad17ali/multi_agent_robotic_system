#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import cv2
import pyrealsense2 as rs
import numpy as np
import scipy.io
import cv2.aruco as aruco
import time
import scipy.io
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

global s, id_list, Coordinates_list,z_anglist,ite,initial_time,move1,move2,poseid,poseidin,goallist,flaglist,Form,FormErrMat,DistErrMat,PI_error1,PI_error2
s = 0.6
id_list = None
Coordinates_list = None
z_anglist = None
poseid = np.zeros([5,3])
poseidin = np.zeros([5,3])

id_mat = []
Coordinates_mat = []
z_ang_mat = []
poseid_mat = []
rgb_mat = []
time_mat = []
ctime_mat1 = []
c_mat1 = []
dist_mat1 = []
ctime_mat2 = []
c_mat2 = []
dist_mat2 = []
FormErrMat = []
DistErrMat = []
ite = 0
initial_time=0
flaglist = [0,0,0,0,0]
PI_error1 = [0,0,0,0]
PI_error2 = [0,0,0,0]

goallist = [[0.6,1]]
Form = [[0.4,0],[-0.4,0]]

# Commanded velocity 
move1 = Twist() # defining the variable to hold values
move1.linear.x = 0
move1.linear.y = 0
move1.linear.z = 0
move1.angular.x = 0
move1.angular.y = 0
move1.angular.z = 0

move2 = Twist() # defining the variable to hold values
move2.linear.x = 0
move2.linear.y = 0
move2.linear.z = 0
move2.angular.x = 0
move2.angular.y = 0
move2.angular.z = 0


class BasicController:
    global ite
    def __init__(self):       
        self.bridge = CvBridge()
        self.rgb_sub = rospy.Subscriber('/realsense/camera/rgb/image_raw', Image, self.coords)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.target_reached = False
        self.target_tolerance = 0.1  # Tolerance for considering the target reached
        self.linear_speed = 0.2  # Linear speed of the robot
        self.angular_speed = 0.5  # Angular speed of the robot
        self.target_x = None
        self.target_y = None
        self.target_orientation = 0
        self.start_list = []
        self.target_list = []

    def coords(self,msg):
        global ite,flaglist,initial_time,initial_time,move1,move2,move3,ctime_mat1,c_mat1,dist_mat1,ctime_mat2,c_mat2,dist_mat2,RelPosition,poseidin, goallist,Form,FormErrMat,DistErrMat,PI_error1,PI_error2
        
        if ite == 0:
            initial_time = time.time()
            time_mat.append(initial_time)   
        try:
            rospy.loginfo(rospy.get_caller_id() + "Recieving RGB data")
            rgb_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            cv2.waitKey(1)
        
    
            camera_msg = rospy.wait_for_message('/realsense/camera/rgb/camera_info', CameraInfo)
            depth_msg = rospy.wait_for_message('/realsense/camera/depth/image_raw', Image)
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg,desired_encoding='passthrough')
    
            camera_matrix = np.array(camera_msg.K).reshape((3,3))
            dist_coeff = np.array(camera_msg.D)
            intrinsics = rs.intrinsics()
            intrinsics.width = 640
            intrinsics.height = 480
            intrinsics.fx = camera_matrix[0,0]
            intrinsics.fy = camera_matrix[1,1]
            intrinsics.ppx = camera_matrix[0,2]
            intrinsics.ppy = camera_matrix[1,2]
    
    
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
            aruco_params = cv2.aruco.DetectorParameters()
            
            aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, aruco_dict, parameters=aruco_params)
    
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
                            start = [start_x,start_y]
                            self.start_list.append(start)
                            
                    
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
                            target = [target_x,target_y]
                            self.target_list.append(target)
                    
                
            cv2.aruco.drawDetectedMarkers(rgb_image, corners)
            cv2.imshow("Camera Feed with ArUco Tags", rgb_image)
                cv2.waitKey(1)
        except Exception as e:
                print(e)
        #start = [start_x,start_y]
        
        #coords.append
        
        #return start,target


    
    
    def coordinates(self, calib_coords,x_,y_,theta):
        x1,y1,x2,y2 = calib_coords
        tan = np.tan(theta)
        cos = np.cos(theta)
        sec = 1/(np.cos(theta))
        cosec = 1/(np.sin(theta))
        y_tmp = (y_ - y1 - (x_- x1)*tan)*sec
        x_tmp = (x_ - x1)*sec + y_tmp*tan
        s_ = np.sqrt((y2-y1)**2 + (x2-x1)**2)
        dist_fac = s_/s
        x = x_tmp/dist_fac
        y = y_tmp/dist_fac
        return x,y
    
    def angle(self, y1_,y2_,x1_,x2_):       
        tmp = float(y2_-y1_)/float(x2_ - x1_)
        invtan = np.arctan(tmp)
        return invtan
    

    
    ##changed controller
    
    def target(self):
        if self.target_list:
            target = self.target_list[-1]
            self.target_x = target[0]
            self.target_y = target[1]
        else:
            rospy.logwarn("No target detected yet.")
    
    def odom_callback(self, data):
        # Extract robot's position and orientation from odometry data
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.robot_x = Coordinates_mat[-1][0]
        self.robot_y = Coordinates_mat[-1][1]
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
    
    def rotate_to_orientation(self):
        ##
        twist_msg = Twist()
        angular_tolerance = 0.05  # Tolerance for considering the target orientation reached
    
        while not rospy.is_shutdown():
            angle_to_target = self.calculate_angle_to_target()
            angle_difference = angle_to_target - self.robot_yaw
    
            # Ensure the angle difference is within the range of -pi to pi
            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi
    
            # Check if the absolute angle difference is within the tolerance
            if abs(angle_difference) < angular_tolerance:
                break  # Target orientation reached
    
            # Set the angular speed based on the sign of the angle difference
            twist_msg.angular.z = self.angular_speed if angle_difference > 0 else -self.angular_speed
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)  # Wait for a short duration

        # Stop rotating
        twist_msg.angular.z = 0
        self.cmd_vel_pub.publish(twist_msg)
        ##
    def calculate_angle_to_target(self):
        angle_to_target = math.atan2(self.target_y - self.robot_y, self.target_x - self.robot_x)
        return angle_to_target

    def navigate_to_target(self):
        angle_to_target = self.calculate_angle_to_target()
        self.rotate_to_orientation()

        distance_to_target = math.sqrt((self.target_x - self.robot_x)**2 + (self.target_y - self.robot_y)**2)
        self.move_forward(distance_to_target)

        # Check if the target is reached
        if distance_to_target < self.target_tolerance:
            self.target_reached = True

    def display_camera_feed(self):
        rate = rospy.Rate(10)  # Rate at which to display the camera feed (adjust as needed)
        while not rospy.is_shutdown():
            try:
                rospy.loginfo(rospy.get_caller_id() + "Receiving RGB data")
                rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.waitKey(1)
    
                # Perform ArUco tag detection
                corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, aruco_dict, parameters=aruco_params)
    
                if ids is not None:
                    # Draw detected markers on the image
                    cv2.aruco.drawDetectedMarkers(rgb_image, corners, ids)
    
                # Display the image with ArUco tags
                cv2.imshow("Camera Feed with ArUco Tags", rgb_image)
                cv2.waitKey(1)
    
            except Exception as e:
                rospy.logerr("Error displaying camera feed: {}".format(e))
    
            rate.sleep()
    
        cv2.destroyAllWindows()
      




if __name__ == '__main__':
    try:
        rospy.init_node('camera_display_node', anonymous=True)
        controller = BasicController()

        # Start a new thread for the BasicController
        controller_thread = threading.Thread(target=controller.run)
        controller_thread.start()

        # Start a new thread to display the camera feed
        camera_display_thread = threading.Thread(target=display_camera_feed)
        camera_display_thread.start()

        rospy.spin()  # Keep the main thread alive

    except rospy.ROSInterruptException:
        pass
