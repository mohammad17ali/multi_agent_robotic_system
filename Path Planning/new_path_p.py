#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import scipy.io
import cv2.aruco as aruco
import time
import scipy.io
from geometry_msgs.msg import Twist
import heapq

global s, id_list, Coordinates_list,z_anglist,ite,initial_time,move,flag,xin,yin,zangin,x,y,zangle,dist,poseid
s = 0.5
id_list = None
Coordinates_list = None
z_anglist = None
poseid = None

id_mat = []
Coordinates_mat = []
z_ang_mat = []
rgb_mat = []
time_mat = []
ctime_mat = []
c_mat = []
dist_mat = []
ite = 0
initial_time=0
flag = 0
dist = None

# Commanded velocity 
move = Twist() # defining the variable to hold values
move.linear.x = 0
move.linear.y = 0
move.linear.z = 0
move.angular.x = 0
move.angular.y = 0
move.angular.z = 0

class realsense:
    global ite
    def __init__(self):
       rospy.init_node('realsense_subscriber_rgb',anonymous=True)

        self.start_pose_pub = rospy.Publisher('start_pose', PoseStamped, queue_size=10)
        self.target_pose_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.start_pose = PoseStamped()  # Initialize the start pose message
        self.target_pose = PoseStamped()
       
       self.bridge = CvBridge()
       #self.camera_info_sub = rospy.Subscriber('/realsense/camera/rgb/camera_info', CameraInfo, self.CameraCallback)
       #self.depth_sub = rospy.Subscriber('/realsense/camera/depth/image_raw', Image, self.IntelSubscriberDepth)
       self.rgb_sub = rospy.Subscriber('/realsense/camera/rgb/image_raw', Image, self.IntelSubscriberRGB)
       #print("RGB", self.rgb_image," Depth ",self.depth_image)
       #self.ArucoDetector()
    
    
    def aruco_coords(self,msg):
       global ite,flag,initial_time,flag,initial_time,xin,yin,zangin,move,ctime_mat,c_mat,dist_mat,RelPosition
       if ite == 0:
          initial_time = time.time()
          time_mat.append(initial_time)   
       try:
          rospy.loginfo(rospy.get_caller_id() + "Recieving RGB data")
          rgb_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
          #cv2.imshow("RGB", rgb_image)
          cv2.waitKey(1)
       except Exception as e:
          print(e)
          
       camera_msg = rospy.wait_for_message('/realsense/camera/rgb/camera_info', CameraInfo)
       depth_msg = rospy.wait_for_message('/realsense/camera/depth/image_raw', Image)
       depth_image = self.bridge.imgmsg_to_cv2(depth_msg,desired_encoding='passthrough')
       
       camera_matrix = np.array(camera_msg.K).reshape((3,3))
       dist_coeff = np.array(camera_msg.D)
       intrinsics = rs.intrinsics()
       intrinsics.width = 640
       intrinsics.height = 480
       intrinsics.fx = camera_matrix[0,0]
       intrinsics.fy = camera_matrix[1,1]
       intrinsics.ppx = camera_matrix[0,2]
       intrinsics.ppy = camera_matrix[1,2]
       #print(intrinsics)
       
       aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
       aruco_params = cv2.aruco.DetectorParameters()
       aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
       
       corners, ids, _ = aruco_detector.detectMarkers(rgb_image)
       #print("corners ", corners, " ids ",ids)
       #print("RGB", rgb_image," Depth ",depth_image, " Camera ", camera_matrix)
       if ids is not None:
          for i in range(len(ids)):
             if ids[i,0] == 1:
                rvec, tvec,z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix, None)
                #print("rvec ",rvec, "tvec ", tvec,"z_ang",z_ang)
                center = np.mean(corners[i][0], axis=0).astype(int)
                p1 = center
                #print("cen",center)
                depth_value = depth_image[center[1], center[0]]
                
                
                if depth_value > 0:
                   #print("dep",depth_value)
                   depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)
                   #print("depp1",depth_point)
                   x1_ = depth_point[0]
                   y1_ = depth_point[1]
                   z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                   org_a = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
             
             elif ids[i,0] == 2:
                rvec, tvec, z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix, None)
                center = np.mean(corners[i][0], axis=0).astype(int)
                depth_value = depth_image[center[1], center[0]]
                p2 = center
                
                if depth_value > 0:
                   depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)
                   #print("depp2",depth_point)
                   x2_ = depth_point[0]
                   y2_ = depth_point[1]
                   z2 = depth_point[2]
                   org_b = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
          #print(x1_," ",y1_," ",x2_," ",y2_)
          x1_, y1_, x2_,y2_ = round(x1_,2),round(y1_,2), round(x2_,2), round(y2_,2)
          calib_coords = [x1_, y1_, x2_,y2_]
          theta = self.angle(y1_,y2_,x1_,x2_)

          if (1 in ids) and (2 in ids):
              x1,y1 = self.coordinates(calib_coords,x1_,y1_,theta)
              x2,y2 = self.coordinates(calib_coords,x2_,y2_,theta)
              marker_text_a = "A({:.2f}, {:.2f})".format( x1, y1)
              marker_text_b = "B({:.2f}, {:.2f})".format(x2, y2)
              cv2.line(rgb_image, p1, p2,(255,0,0), 1)
              #cv2.putText(color_image, marker_text_a, org_a, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
              #cv2.putText(color_image, marker_text_b, org_b, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
          else:
              print("Reference tags are not in view")
       id_list = []
       Coordinates_list = []
       z_anglist = []
       poseid = []                 
       if ids is not None:
          for i in range(len(ids)):
             rvec, tvec, z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix,None)
             center = np.mean(corners[i][0], axis=0).astype(int)
             depth_value = depth_image[center[1], center[0]]
             
             if depth_value > 0:
                depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)

                marker_text = "Marker ID: {} | Coordinates: {:.2f}, {:.2f}, {:.2f}".format(ids[i][0], depth_point[0], depth_point[1], depth_point[2])
                   
                marker_x_ = depth_point[0]
                marker_y_ = depth_point[1]
                marker_z = depth_point[2]
                  
                #using the calibrated values
                marker_x,marker_y = self.coordinates(calib_coords,marker_x_,marker_y_,theta)
                id_list.append(ids[i,0])
                Coordinates_list.append([marker_x,marker_y])
                
                z_ang = z_ang-np.pi/2 
                  
                if z_ang<0:
                   z_ang = z_ang+2*np.pi
                z_anglist.append(z_ang*180/np.pi)
                
                poseid.append([ids[i,0],marker_x,marker_y,z_ang])
                
                if ids[i,0] == 3 and flag == 0:
                    
                   print("yes")
                   xin = marker_x
                   yin = marker_y
                   zangin = z_ang
                   flag = 1
                   start_pose = [xin,yin,0]
                elif ids[i,0] == 4 and flag == 1:
                   x = marker_x
                   y = marker_y
                   zangle = z_ang
                   target_pose = [x,y,0]
                 
                #print(ids[i]," ", marker_x, " ",marker_y)
                marker_text = "ID: {} ({:.2f}, {:.2f})".format(ids[i][0], marker_x, marker_y)
                # Convert coordinates to integers before passing to putText
                org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)

                cv2.putText(rgb_image, marker_text, org,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.aruco.drawDetectedMarkers(rgb_image, corners)
                 self.publish_start_pose(start_pose)
                 self.publish_target_pose(target_pose)
                 self.rate.sleep()
                
       print("id list",id_list,"Coordinates list",Coordinates_list,"z_anglist",z_anglist)
       cv2.imshow("ArUco Marker Detection", rgb_image)
       # Exit the program when the 'Esc' key is pressed
       #if cv2.waitKey(1) & 0xFF == 27:
          #break
       RelPosition = self.RelPose(poseid)
       print(RelPosition)
          
       id_mat.append(id_list)
       Coordinates_mat.append(Coordinates_list)
       z_ang_mat.append(z_anglist)
       #rgb_mat.append(rgb_image)
       
       current_time = time.time()
       #print(" xin ", xin," yin ", yin," zangin ", zangin," x ", x," y ", y," zang ", zangle)
       print("ite ",ite," initial time ",initial_time," current time ",current_time)
       
       ite = ite+1
       
       time_mat.append(current_time)
       self.controller(x,y,zangle,xin,yin,zangin,ctime_mat,c_mat,dist_mat)
       print("x",x,"xin", xin, "y",y,"yin",yin," zangin ", zangin,"dist",dist,"goalx",goalx,"goaly",goaly,"zvel",move.angular.z)
       
       if (current_time > initial_time + 8 ):
          id_mat1 = np.array(id_mat)
          Coordinates_mat1 = np.array(Coordinates_mat)
          z_ang_mat1 = np.array(z_ang_mat)
          #rgb_mat1 = np.array(rgb_mat)
          time_mat1 = np.array(time_mat)
          c_mat1 = np.array(c_mat)
          ctime_mat1 = np.array(ctime_mat)
          dist_mat1 = np.array(dist_mat)
          scipy.io.savemat('Aruco.mat', dict(idmat=id_mat1, Coodmat=Coordinates_mat1, zmat=z_ang_mat1, cmat=c_mat1, ctimemat = ctime_mat1, timemat = time_mat1,distmat = dist_mat1))

    def publish_start_pose(self, start_pose):
        # Update the start pose message
        self.start_pose.header.stamp = rospy.Time.now()
        self.start_pose.pose = start_pose
        # Publish the start pose message
        self.start_pose_pub.publish(self.start_pose)
        rospy.loginfo("Published start pose: {}".format(start_pose))

    def publish_target_pose(self, target_pose):
        # Update the target pose message
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.pose = target_pose
        # Publish the target pose message
        self.target_pose_pub.publish(self.target_pose)
        rospy.loginfo("Published target pose: {}".format(target_pose))
        
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
    
    def my_estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
       marker_points = np.array([[-marker_size / 2, marker_size / 2, 0], [marker_size / 2, marker_size / 2, 0], [marker_size / 2, -marker_size / 2, 0], [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
       trash = []
       rvecs = []
       tvecs = []
       z_angle = 0
       for c in corners:
          nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
          RR,_ = cv2.Rodrigues(R)
          sy = np.sqrt(RR[0,0]*RR[0,0]+RR[1,0]*RR[1,0])
          if sy>1e-6:
              z_angle = z_angle+np.arctan2(RR[1,0],RR[0,0])
          else:
              z_angle = z_angle
          
          rvecs.append(R)
          tvecs.append(t)
          trash.append(nada)
       #z_angle = z_angle/4
       return rvecs, tvecs, z_angle, trash
    
    
       
    def RelPos(self, poseid):
       RelPosition = None
       pose_sort = poseid[poseid[:, 0].argsort()]
       for i in range(len(pose_sort)):
          for j in range(len(pose_sort)):
             RelPosition[i*len(pose_sort)+j,0] = pose_sort[i,1]-pose_sort[j,1]
             RelPosition[i*len(pose_sort)+j,1] = pose_sort[i,2]-pose_sort[j,2]
       return RelPosition  
'''
##Path Planning (D*lite)

class BasicPathPlanner:
    def __init__(self, grid_width, grid_height):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.obstacle_map = np.zeros((grid_width, grid_height), dtype=int)

    def update_obstacle_map(self, lidar_data):
        # Update obstacle map based on Lidar sensor data
        # lidar_data: list of 360 proximity values representing obstacles
        for angle, distance in enumerate(lidar_data):
            x = int(distance * np.cos(np.radians(angle)))  # Convert polar coordinates to Cartesian
            y = int(distance * np.sin(np.radians(angle)))
            if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                self.obstacle_map[x, y] = 1  # Mark obstacle in obstacle map

    def plan_path(self, start, target):
        # A* path planning algorithm
        # start: tuple (x, y) representing start point
        # target: tuple (x, y) representing target point
        # Returns: list of tuples representing path from start to target
        # Placeholder implementation, replace with actual A* algorithm
        
        # Here's a placeholder implementation returning a straight-line path from start to target
        path = []
        current = start
        while current != target:
            path.append(current)
            x_diff = target[0] - current[0]
            y_diff = target[1] - current[1]
            if x_diff > 0:
                current = (current[0] + 1, current[1])
            elif x_diff < 0:
                current = (current[0] - 1, current[1])
            elif y_diff > 0:
                current = (current[0], current[1] + 1)
            else:
                current = (current[0], current[1] - 1)
        path.append(target)
        return path
'''
## Motion Controller
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from math import atan2, sqrt

class Controller:
    def __init__(self):
        self.start_pose = None
        self.target_pose = None
        self.current_pose = None
        rospy.Subscriber('target_pose', PoseStamped, self.handle_pose)
        rospy.Subscriber('current_pose', PoseStamped, self.update_current_pose)
        self.velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def handle_pose(self, pose):
        if self.start_pose is None:
            # Set start pose
            self.start_pose = pose
            print("Received start pose:", self.start_pose)
        else:
            print("Ignoring additional pose:", pose)

    def update_current_pose(self, pose):
        self.start_pose = pose
        self.current_pose = pose

    def move_to_target(self):
        if self.start_pose is None or self.target_pose is None:
            print("Missing start or target pose. Cannot move.")
            return
        print("Moving from", self.start_pose, "to", self.target_pose)
        
        while not rospy.is_shutdown():
            if self.current_pose is not None:
                current_x = self.current_pose.pose.position.x
                current_y = self.current_pose.pose.position.y
                target_x = self.target_pose.pose.position.x
                target_y = self.target_pose.pose.position.y

                distance_to_target = sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

                if distance_to_target < 0.1:
                    print("Reached target pose")
                    break

                angle_to_target = atan2(target_y - current_y, target_x - current_x)

                move_cmd = Twist()
                move_cmd.linear.x = 0.2  # Constant linear velocity
                move_cmd.angular.z = 0.5 * angle_to_target  # Adjusting angular velocity based on angle to target
                self.velocity_pub.publish(move_cmd)
            
            self.rate.sleep()

        # Reset start and target poses after completing the motion
        self.start_pose = None
        self.target_pose = None

##
if __name__ == '__main__':
    try:
        rospy.init_node('controller_node', anonymous=True)
        #planner = BasicPathPlanner(grid_width=100, grid_height=100)  # Example grid size
        controller = Controller()
        subscriber = IntelSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
##
if __name__ == '__main__':
    try:
       velocity_pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
       rs_subscriber = IntelSubscriber()
       #print(rs_subscriber.rgb_image)
       rate = rospy.Rate(5)
       while not rospy.is_shutdown():
          velocity_pub.publish(move)
          rate.sleep()
       rospy.spin()
    except rospy.ROSInterruptException:
       pass
