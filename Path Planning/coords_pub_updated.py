#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import Point,Pose
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt

class CoordsPublisher:
    def __init__(self):
        rospy.init_node('coords_publisher',anonymous=True)
        self.start_pub = rospy.Publisher('/start_pub', Point, queue_size=10)
        self.target_pub = rospy.Publisher('/target_pub', Point, queue_size=10)
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.cfg)
        #self.pipeline.start(self.cfg)
       
        self.depth_intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.color_intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
       
    def capture_and_publish(self):
        x1_ = y1_ = x2_ = y2_ = start_x_ = start_y_ = target_x_ = target_y_ = 0
        org_start = [0,0]
        org_target = [1,1]
        p1 = [1,1]
        p2 = [0,0]
        start = [1,1,0]
        target = [0,0,0]
        s = 0.6
        try:
            while not rospy.is_shutdown():
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not color_frame or not depth_frame:
                    continue
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
    
                ##Processing
                aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
                aruco_params = cv2.aruco.DetectorParameters()
                aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
           
                corners, ids, _ = aruco_detector.detectMarkers(color_image)
                                
                if ids is not None:
                    for i in range(len(ids)):
                        if ids[i] ==1:
                            #ind = ids.index(id)
                            rvec, tvec, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                            center = np.mean(corners[i][0], axis=0).astype(int)
                            depth_value = depth_frame.get_distance(center[0], center[1])
                            p1 = center
                            if depth_value > 0:
                                depth_point = rs.rs2_deproject_pixel_to_point(
                                    depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
                                x1_ = depth_point[0]
                                y1_ = depth_point[1]
                                z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                                org_a = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
    
                        elif ids[i] ==2:
                            #ind = ids.index(id)
                            rvec, tvec, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                            center = np.mean(corners[i][0], axis=0).astype(int)
                            depth_value = depth_frame.get_distance(center[0], center[1])
                            p2 = center
                            if depth_value > 0:
                                depth_point = rs.rs2_deproject_pixel_to_point(
                                    depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
    
                                x2_ = depth_point[0]
                                y2_ = depth_point[1]
                                z2 = depth_point[2]
                                org_b = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10) 
                        elif ids[i] ==3:
                            #ind = ids.index(id)
                            rvec, tvec, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                            center = np.mean(corners[i][0], axis=0).astype(int)
                            depth_value = depth_frame.get_distance(center[0], center[1])
                            start_center = center
                            if depth_value > 0:
                                depth_point = rs.rs2_deproject_pixel_to_point(
                                    depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
    
                                start_x_ = depth_point[0]
                                start_y_ = depth_point[1]
                                start_z = depth_point[2]
                                org_start = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
                        elif ids[i] ==4:
                            #ind = ids.index(id)
                            rvec, tvec, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                            center = np.mean(corners[i][0], axis=0).astype(int)
                            depth_value = depth_frame.get_distance(center[0], center[1])
                            target_center = center
                            if depth_value > 0:
                                depth_point = rs.rs2_deproject_pixel_to_point(
                                    depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
    
                                target_x_ = depth_point[0]
                                target_y_ = depth_point[1]
                                target_z = depth_point[2]
                                org_target = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10) 
                        else:
                            pass
                    x1_, y1_, x2_,y2_ = round(x1_,2),round(y1_,2), round(x2_,2), round(y2_,2)
                    calib_coords = [x1_, y1_, x2_,y2_]
                    theta = self.angle(y1_,y2_,x1_,x2_)
    
                    if (1 in ids) and (2 in ids):
                        x1,y1 = self.coordinates(calib_coords,x1_,y1_,theta)
                        x2,y2 = self.coordinates(calib_coords,x2_,y2_,theta)
                        marker_text_a = "A({:.2f}, {:.2f})".format( x1, y1)
                        marker_text_b = "B({:.2f}, {:.2f})".format(x2, y2)
                        cv2.line(color_image, p1, p2,(255,0,0), 1)
                        cv2.putText(color_image, marker_text_a, org_a, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(color_image, marker_text_b, org_b, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    else:
                        break
    
                    if (3 in ids) and (4 in ids):
                        start_x,start_y = self.coordinates(calib_coords,start_x_,start_y_,theta)
                        start_point = Point(start_x,start_y,start_z) #assigning start point
                        
                        target_x,target_y = self.coordinates(calib_coords,target_x_,target_y_,theta)
                        target_point = Point(target_x,target_y,target_z) #assigning target point
                        
                        marker_text_start = "S({:.2f}, {:.2f})".format( start_x, start_y)
                        marker_text_target = "T({:.2f}, {:.2f})".format(target_x, target_y)
                        
                        cv2.line(color_image, org_start, org_target,(0,0,255), 1)
                        cv2.putText(color_image, marker_text_start, org_start, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(color_image, marker_text_target, org_target, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                    else:
                        break
                        
                    cv2.aruco.drawDetectedMarkers(color_image, corners)
                self.start_pub.publish(start_point)
                rospy.loginfo("Publishing Start Coordinates")
                print(start_point)
                self.target_pub.publish(target_point)
                rospy.loginfo("Publishing Target Coordinates")
                print(target_point)
                #self.rate.sleep()
                
                
                cv2.imshow("Live Feed of Surface", color_image)
        finally:
            self.pipeline.stop()
            #cv2.destroyAllWindows()
            
            
            
    def coordinates(self, calib_coords,x_,y_,theta):
        s = 0.6
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
        return rvecs, tvecs, trash

'''------------------------------------------------------------------------------------'''

if __name__ == '__main__':
    try:
        coords_publisher = CoordsPublisher()
        coords_publisher.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
