#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np

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
        x1_ = y1_ = x2_ = y2_ = 0
        org_start = [0,0]
        org_target = [0,0]
        p1 = [0,0]
        p2 = [0,0]
        start = [0,0,0]
        target = [0,0,0]
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
                    for id in ids:
                        if id ==1:
                            ind = ids.index(id)
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[ind], 0.05, np.eye(3), None)
                            center = np.mean(corners[ind][0], axis=0).astype(int)
                            depth_value = depth_frame.get_distance(center[0], center[1])
                            p1 = center
                            if depth_value > 0:
                                depth_point = rs.rs2_deproject_pixel_to_point(
                                    depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
                                x1_ = depth_point[0]
                                y1_ = depth_point[1]
                                z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                                org_a = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
    
                        elif id ==2:
                            ind = ids.index(id)
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[ind], 0.05, np.eye(3), None)
                            center = np.mean(corners[ind][0], axis=0).astype(int)
                            depth_value = depth_frame.get_distance(center[0], center[1])
                            p2 = center
                            if depth_value > 0:
                                depth_point = rs.rs2_deproject_pixel_to_point(
                                    depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
    
                                x2_ = depth_point[0]
                                y2_ = depth_point[1]
                                z2 = depth_point[2]
                                org_b = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10) 
                        elif id ==3:
                            ind = ids.index(id)
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[ind], 0.05, np.eye(3), None)
                            center = np.mean(corners[ind][0], axis=0).astype(int)
                            depth_value = depth_frame.get_distance(center[0], center[1])
                            start_center = center
                            if depth_value > 0:
                                depth_point = rs.rs2_deproject_pixel_to_point(
                                    depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
    
                                start_x_ = depth_point[0]
                                start_y_ = depth_point[1]
                                start_z = depth_point[2]
                                org_start = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
                        elif id ==4:
                            ind = ids.index(id)
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[ind], 0.05, np.eye(3), None)
                            center = np.mean(corners[ind][0], axis=0).astype(int)
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
    
                    if (3 in ids) and (4 in ids):
                        start_x,start_y = coordinates(calib_coords,start_x_,start_y_,theta)
                        start = [start_x,start_y,start_z] #assigning start point
                        
                        target_x,target_y = coordinates(calib_coords,target_x_,target_y_,theta)
                        target = [target_x,target_y,target_z] #assigning target point
                        
                        marker_text_start = "S({:.2f}, {:.2f})".format( start_x, start_y)
                        marker_text_target = "T({:.2f}, {:.2f})".format(target_x, target_y)
                        
                        cv2.line(color_image, org_start, org_target,(0,0,255), 1)
                        cv2.putText(color_image, marker_text_start, org_start, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(color_image, marker_text_target, org_target, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                    else:
                        break
                        
                    cv2.aruco.drawDetectedMarkers(rgb_image, corners)
                self.start_pub.publish(start)
                self.target_pub.publish(target)
                self.rate.sleep()
                rospy.loginfo("Publishing Coordinates")
                
                cv2.imshow("ArUco Marker Detection", color_image)
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

'''------------------------------------------------------------------------------------'''

if __name__ == '__main__':
    try:
        coords_publisher = CoordsPublisher()
        coords_publisher.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
