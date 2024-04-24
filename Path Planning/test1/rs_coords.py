#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters_create()

class RealSenseCameraNode:
    def __init__(self):
        rospy.init_node('realsense_camera_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        self.camera = cv2.VideoCapture(0)  # Change index if necessary
        self.x1 = None
        self.x1_ = None
        self.x1__ = None
        self.x2 = None
        self.x2_ = None
        self.x2__ = None
        self.y1 = None
        self.y1_ = None
        self.y1__ = None
        self.y2 = None
        self.y2_ = None
        self.y2__ = None
        self.theta = None
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.start_x_ = None
        self.start_y_ = None
        self.start_z = None
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
            
            aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, aruco_dict, parameters=aruco_params)
    
            if ids is not None:
                for i in range(len(ids)):
                    if ids[i] == 1:
                        a_corner = corners[i]
                        p1 = np.mean(a_corner[0], axis=0).astype(int)
                        depth_value = depth_frame.get_distance(p1[0], p1[1])
                        if depth_value > 0:
                            depth_point = rs.rs2_deproject_pixel_to_point(
                                depth_frame.profile.as_video_stream_profile().intrinsics, [p1[0], p1[1]], depth_value)
                            self.x1__ = depth_point[0]
                            self.y1__ = depth_point[1]
                            self.z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                            org_a = (int(a_corner[0, 0, 0]), int(a_corner[0, 0, 1]) - 10)
                    if ids[i] == 2:
                        b_corner = corners[i]
                        p2 = np.mean(b_corner[0], axis=0).astype(int)
                        depth_value = depth_frame.get_distance(p2[0], p2[1])
                        if depth_value > 0:
                            depth_point = rs.rs2_deproject_pixel_to_point(
                                depth_frame.profile.as_video_stream_profile().intrinsics, [p2[0], p2[1]], depth_value)
                            self.x2__ = depth_point[0]
                            self.y2__ = depth_point[1]
                            self.z2 = depth_point[2] # no need for separate z1_, as its value won't be changed
                            org_b = (int(b_corner[0, 0, 0]), int(b_corner[0, 0, 1]) - 10) 
    
                    self.x1_, self.y1_, self.x2_,self.y2_ = round(self.x1__,2),round(self.y1__,2), round(self.x2__,2), round(self.y2__,2)
                    calib_coords = [self.x1_, self.y1_, self.x2_,self.y2_]
                    theta = angle(self.y1_,self.y2_,self.x1_,self.x2_)
    
                    if (1 in ids) and (2 in ids):
                        self.x1,self.y1 = coordinates(calib_coords,self.x1_,self.y1_,self.theta)
                        self.x2,self.y2 = coordinates(calib_coords,self.x2_,self.y2_,self.theta)
                        marker_text_a = "A({:.2f}, {:.2f})".format( self.x1, self.y1)
                        marker_text_b = "B({:.2f}, {:.2f})".format(self.x2, self.y2)
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
                            self.start_x_ = depth_point[0]
                            self.start_y_ = depth_point[1]
                            self.start_z = depth_point[2] # no need for separate z1_, as its value won't be changed
                            start_org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
                            
                            self.start_x, self.start_y = coordinates(calib_coords, self.start_x_,self.start_y_, self.theta)
                            marker_text_start = "P1({:.2f}, {:.2f})".format( self.self.start_x, self.start_y)
                            cv2.putText(color_image, marker_text_start, start_org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            start = [self.start_x,self.start_y]
                            self.start_list.append(start)
                            
                    
                    elif ids[i] == 3: #id30 = target point tag
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                        center = np.mean(corners[i][0], axis=0).astype(int)
                        depth_value = depth_frame.get_distance(center[0], center[1])
    
                        if depth_value > 0:
                            depth_point = rs.rs2_deproject_pixel_to_point(
                                depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
                            self.target_x_ = depth_point[0]
                            self.target_y_ = depth_point[1]
                            self.target_z = depth_point[2] # no need for separate z1_, as its value won't be changed
                            target_org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
                            
                            self.target_x, self.target_y = coordinates(calib_coords, self.target_x_,self.target_y_, self.theta)
                            marker_text_target = "P2({:.2f}, {:.2f})".format( self.target_x, self.target_y)
                            cv2.putText(color_image, marker_text_target, target_org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            target = [self.target_x,self.target_y]
                            self.target_list.append(target)
                    
                
            cv2.aruco.drawDetectedMarkers(rgb_image, corners)
            cv2.imshow("Camera Feed with ArUco Tags", rgb_image)
                cv2.waitKey(1)
            except Exception as e:
                    print(e)
        
    
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
    

if __name__ == '__main__':
    try:
        camera_node = RealSenseCameraNode()
        rospy.Subscriber('/camera/rgb/image_raw', Image, camera_node.coords)
        rospy.spin()  # Keep the node running until terminated
    except rospy.ROSInterruptException:
        pass

