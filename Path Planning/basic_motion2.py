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

class Server:
    def __init__(self,ID):
       self.ID = 
    
    def lidar_callback(self, msg):
    def odom_callback(self, msg):
    
class IntelSubscriber:
    global ite
    def __init__(self):       
       self.bridge = CvBridge()
       self.rgb_sub = rospy.Subscriber('/realsense/camera/rgb/image_raw', Image, self.IntelSubscriberRGB)
    
    def IntelSubscriberRGB(self,msg):
       global ite,flaglist,initial_time,initial_time,move1,move2,move3,ctime_mat1,c_mat1,dist_mat1,ctime_mat2,c_mat2,dist_mat2,RelPosition,poseidin, goallist,Form,FormErrMat,DistErrMat,PI_error1,PI_error2
       if ite == 0:
          initial_time = time.time()
          time_mat.append(initial_time)   
       try:
          rospy.loginfo(rospy.get_caller_id() + "Recieving RGB data")
          rgb_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
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
       
       aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
       aruco_params = cv2.aruco.DetectorParameters()
       aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
       
       corners, ids, _ = aruco_detector.detectMarkers(rgb_image)
       if ids is not None:
          for i in range(len(ids)):
             if ids[i,0] == 1:
                rvec, tvec,z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix, None)
                center = np.mean(corners[i][0], axis=0).astype(int)
                p1 = center
                depth_value = depth_image[center[1], center[0]]
                
                
                if depth_value > 0:
                   depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)
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
                   x2_ = depth_point[0]
                   y2_ = depth_point[1]
                   z2 = depth_point[2]
                   org_b = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
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
       poseid = np.empty([5,3])                 
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
                
                # Robot ids
                
                if flaglist[ids[i,0]-1] == 0:
                   poseidin[ids[i,0]-1,0] = marker_x
                   poseidin[ids[i,0]-1,1] = marker_y
                   poseidin[ids[i,0]-1,2] = z_ang
                   flaglist[ids[i,0]-1] = 1
                elif flaglist[ids[i,0]-1] == 1:
                   poseid[ids[i,0]-1,0] = marker_x
                   poseid[ids[i,0]-1,1] = marker_y
                   poseid[ids[i,0]-1,2] = z_ang
                
                marker_text = "ID: {} ({:.2f}, {:.2f})".format(ids[i][0], marker_x, marker_y)
                # Convert coordinates to integers before passing to putText
                org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)

                cv2.putText(rgb_image, marker_text, org,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.aruco.drawDetectedMarkers(rgb_image, corners)
       print("id list",id_list,"Coordinates list",Coordinates_list,"z_anglist",z_anglist)
       print("poseid",poseid,"poseidin",poseidin)   
       cv2.imshow("ArUco Marker Detection", rgb_image)
       # Exit the program when the 'Esc' key is pressed
       #if cv2.waitKey(1) & 0xFF == 27:
          #break
       RelPosition = self.RelPos(poseid)
       print(RelPosition)
          
       id_mat.append(id_list)
       Coordinates_mat.append(Coordinates_list)
       z_ang_mat.append(z_anglist)
       poseid_mat.append(list(poseid))
       #rgb_mat.append(rgb_image)
       
       current_time = time.time()
       print("ite ",ite," initial time ",initial_time," current time ",current_time)
       
       ite = ite+1
       
       time_mat.append(current_time)
       
       self.RelController(poseid[2,:],RelPosition[13,:],Form[0][:],goallist,FormErrMat,DistErrMat,move1,0,c_mat1,ctime_mat1,PI_error1)
       self.RelController(poseid[3,:],RelPosition[17,:],Form[1][:],goallist,FormErrMat,DistErrMat,move2,0,c_mat2,ctime_mat2,PI_error2)
       #self.controller(poseid[4][0],poseid[4][1],poseid[4][2],poseidin[4][0],poseidin[4][1],poseidin[4][2],ctime_mat1,c_mat1,dist_mat1,move1,goallist[0][0],goallist[0][1])
       #self.controller(poseid[2][0],poseid[2][1],poseid[2][2],poseidin[2][0],poseidin[2][1],poseidin[2][2],ctime_mat2,c_mat2,dist_mat2,move2,goallist[1][0],goallist[1][1])
       #self.controller(poseid[3][0],poseid[3][1],poseid[3][2],poseidin[3][0],poseidin[3][1],poseidin[3][2],ctime_mat3,c_mat3,dist_mat3,move3,goallist[2][0],goallist[2][1])
       
       if (current_time > initial_time + 8 ):
          id_mat1 = np.array(id_mat)
          Coordinates_mat1 = np.array(Coordinates_mat)
          z_ang_mat1 = np.array(z_ang_mat)
          #rgb_mat1 = np.array(rgb_mat)
          time_mat1 = np.array(time_mat)
          c_mat11 = np.array(c_mat1)
          ctime_mat11 = np.array(ctime_mat1)
          dist_mat11 = np.array(dist_mat1)
          c_mat22 = np.array(c_mat2)
          ctime_mat22 = np.array(ctime_mat2)
          dist_mat22 = np.array(dist_mat2)
          distErr = np.array(DistErrMat)
          FormErr = np.array(FormErrMat)
          #c_mat33 = np.array(c_mat3)
          #ctime_mat33 = np.array(ctime_mat3)
          #dist_mat33 = np.array(dist_mat3)
          scipy.io.savemat('Aruco.mat', dict(idmat=id_mat1, Coodmat=Coordinates_mat1, zmat=z_ang_mat1, cmat1=c_mat11, ctimemat1 = ctime_mat11, timemat = time_mat1,distmat1 = dist_mat11, cmat2=c_mat22, ctimemat2 = ctime_mat22, distmat2 = dist_mat22,Disterr = distErr,Formerr = FormErr))

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
    
    def controller(self,x,y,zang,xin,yin,zangin,ctime_mat,c_mat,dist_mat,move,goalx,goaly):
       alpha = 0.1
       k = 0.1
       #goalx = 0.6#xin + 0.6*np.cos(zangin+np.pi/4)
       #goaly = 0.6#yin + 0.6*np.sin(zangin+np.pi/4)
       dist = np.sqrt((goalx-x)**2 +(goaly-y)**2)
       desired_orientation = np.arctan2((goaly - y),(goalx - x))
       orientation_error = desired_orientation - zang
       orientation_error%=(2*np.pi)
       if orientation_error>np.pi:
          orientation_error-= 2*np.pi
          
       if np.absolute(dist) > 0.01:
          move.linear.x = alpha*dist
          move.angular.z = -k*orientation_error
       else:
          move.linear.x = 0
          move.angular.z = 0
       ctime = time.time()
       dist_mat.append(dist)
       c_mat.append([move.linear.x,move.angular.z])
       ctime_mat.append(ctime)
       
    def RelController(self,poseid,RelPose,Form,goallist,Formerror,Disterror,move,lead,c_mat,ctime_mat,PI_error):
       dx = Form[0]
       dy = Form[1]
       l = 0.13
       goal_x = goallist[0][0]
       goal_y = goallist[0][1]
       distance_to_goal_x = goal_x-poseid[0]
       distance_to_goal_y = goal_y-poseid[1]
       
       distance_to_goal = np.sqrt(distance_to_goal_x**2+distance_to_goal_y**2)
       Disterror.append(distance_to_goal)
       print("Dist_to_goal",distance_to_goal,"lead",lead)
       
       # Controller gain
       
       if lead==1:
          K_tr = 0.2
          K_tr_I = 0.0001
       else:
          K_tr = 0
          K_tr_I = 0
       K_for_p = 0.1
       K_for_I = 0
       K_ang = 0.1
       
       x_form_error = dx - RelPose[0]
       y_form_error = dy - RelPose[1]
       
       Tot_distance_to_goal_x = PI_error[0]+distance_to_goal_x
       Tot_distance_to_goal_y = PI_error[1]+distance_to_goal_y
       Tot_x_form_error = PI_error[2]+x_form_error
       Tot_y_form_error = PI_error[3]+y_form_error
       print(K_tr," ", lead)
       print("x_error",x_form_error,"y_error",y_form_error,"PIx_error",Tot_x_form_error,"PIy_error",Tot_y_form_error)
       
       x_dot = K_for_p * (x_form_error) + K_tr*distance_to_goal_x + K_for_I * (Tot_x_form_error) + K_tr_I*Tot_distance_to_goal_x
       y_dot = K_for_p * (y_form_error) + K_tr*distance_to_goal_y + K_for_I * (Tot_y_form_error) + K_tr_I*Tot_distance_to_goal_y
       
       distance_to_pose = np.sqrt(x_form_error**2+y_form_error**2)
       Formerror.append(distance_to_pose)
       print("Dist_to_pose",distance_to_pose,"lead",lead)
       
       rob_theta = poseid[2]*180/np.pi
       if(rob_theta >179):
          rob_theta = rob_theta-360
       
       if lead==1:
          if distance_to_pose < 0.05 and distance_to_goal < 0.05:
             move.linear.x = 0
             move.angular.z = 0#K_ang*((rob_theta)*np.pi/180)
          else:
             move.linear.x = (x_dot*np.cos(rob_theta*np.pi/180) + y_dot*np.sin(rob_theta*np.pi/180))
             move.angular.z = -(-x_dot*np.sin(rob_theta*np.pi/180) + y_dot*np.cos(rob_theta*np.pi/180))/(2*l) 
       else:
          if distance_to_pose < 0.05:
             move.linear.x = 0
             move.angular.z = 0#K_ang*((rob_theta)*np.pi/180)
          else:
             move.linear.x = (x_dot*np.cos(rob_theta*np.pi/180) + y_dot*np.sin(rob_theta*np.pi/180))
             move.angular.z = -(-x_dot*np.sin(rob_theta*np.pi/180) + y_dot*np.cos(rob_theta*np.pi/180))/(2*l)
          
       ctime = time.time()
       c_mat.append([move.linear.x,move.angular.z])
       ctime_mat.append(ctime)
       
       PI_error[0]=Tot_distance_to_goal_x
       PI_error[1]=Tot_distance_to_goal_y
       PI_error[2]=Tot_x_form_error
       PI_error[3]=Tot_y_form_error
       
    
    def RelPos(self, poseid):
       r,c = poseid.shape
       RelPosition = np.empty([r*r,2])
       for i in range(r):
          for j in range(r):
             RelPosition[i*r+j,0] = poseid[i,0]-poseid[j,0]
             RelPosition[i*r+j,1] = poseid[i,1]-poseid[j,1]
       return RelPosition  
       

if __name__ == '__main__':
    try:
       rospy.init_node('realsense_subscriber_rgb',anonymous=True)
       
       server1 = Server(2)
       server2 = Server(3)
       server3 = Server(4)
       
       velocity_pub1 = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
       velocity_pub2 = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=10)
       #velocity_pub3 = rospy.Publisher('tb3_2/cmd_vel', Twist, queue_size=10)
       pose_subscriber1 = rospy.Subscriber('tb3_0/odom', Odometry, server1.odom_callback)
       lidar_subscriber1 = rospy.Subscriber('tb3_0/scan', LaserScan, server1.lidar_callback)
       pose_subscriber2 = rospy.Subscriber('tb3_1/odom', Odometry, server2.odom_callback)
       lidar_subscriber2 = rospy.Subscriber('tb3_1/scan', LaserScan, server2.lidar_callback)
       pose_subscriber3 = rospy.Subscriber('tb3_2/odom', Odometry, server3.odom_callback)
       lidar_subscriber3 = rospy.Subscriber('tb3_2/scan', LaserScan, server3.lidar_callback)
       rs_subscriber = IntelSubscriber()
       rate = rospy.Rate(10)
       while not rospy.is_shutdown():
          velocity_pub1.publish(move1)
          velocity_pub2.publish(move2)
          #velocity_pub3.publish(move3)
          rate.sleep()
       rospy.spin()
    except rospy.ROSInterruptException:
       pass
