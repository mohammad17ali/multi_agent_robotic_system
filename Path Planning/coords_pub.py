#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np

class CoordsPublisher:
    def __init__(self):
        self.x1_ = self.y1_ = self.z1_ = x2_= self.y2_ = self.z2_ = 0
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
       
            corners, ids, _ = cv2.aruco.detectMarkers(color_image, aruco_dict, parameters=aruco_params)
            

                
            if ids is not None:
                for i in range(len(ids)):
                    if ids[i] == 1:
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                        center = np.mean(corners[i][0], axis=0).astype(int)
                        p1 = center
                        depth_value = depth_frame.get_distance(center[0], center[1])
                        p1 = center

                        if depth_value > 0:
                            depth_point = rs.rs2_deproject_pixel_to_point(
                                depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
                            self.x1_ = depth_point[0]
                            self.y1_ = depth_point[1]
                            self.z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                            org_a = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)


                    elif ids[i] == 2:
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                        center = np.mean(corners[i][0], axis=0).astype(int)
                        depth_value = depth_frame.get_distance(center[0], center[1])
                        p2 = center
                        if depth_value > 0:
                            depth_point = rs.rs2_deproject_pixel_to_point(
                                depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)

                            self.x2_ = depth_point[0]
                            self.y2_ = depth_point[1]
                            self.z2 = depth_point[2]
                            org_b = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10) 


                    self.x1_, self.y1_, self.x2_,self.y2_ = round(self.x1_,2),round(self.y1_,2), round(self.x2_,2), round(self.self.y2_,2)
                    self.self.calib_coords = [self.x1_, self.y1_, self.x2_,self.y2_]
                    self.theta = angle(self.y1_,self.y2_,self.x1_,self.x2_)

                    if (1 in ids) and (2 in ids):
                        self.x1,self.y1 = coordinates(self.calib_coords,self.x1_,self.y1_,self.theta)
                        self.x2,self.y2 = coordinates(self.calib_coords,self.x2_,self.y2_,self.theta)
                        marker_text_a = "A({:.2f}, {:.2f})".format( self.x1, self.y1)
                        marker_text_b = "B({:.2f}, {:.2f})".format(self.x2, self.y2)
                        cv2.line(color_image, p1, p2,(255,0,0), 1)
                        cv2.putText(color_image, marker_text_a, org_a, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(color_image, marker_text_b, org_b, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    else:
                        break

            if ids is not None:
                for i in range(len(ids)):
                    if ids[i] == 16: #id16 = starting point tag
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

                            self.start_x, self.start_y = coordinates(self.calib_coords, self.start_x_,self.start_y_, self.theta)
                            marker_text_start = "A({:.2f}, {:.2f})".format( start_x, start_y)
                            cv2.putText(color_image, marker_text_start, start_org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            self.start = [self.start_x,self.start_y]

                    elif ids[i] == 20: #id30 = target point tag
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                        center = np.mean(corners[i][0], axis=0).astype(int)
                        depth_value = depth_frame.get_distance(center[0], center[1])

                        if depth_value > 0:
                            depth_point = rs.rs2_deproject_pixel_to_point(
                                depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
                            self.target_x_ = depth_point[0]
                            self.target_y_ = depth_point[1]
                            self.target_x = depth_point[2] # no need for separate z1_, as its value won't be changed
                            target_org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)

                            self.target_x, self.target_y = coordinates(self.calib_coords, self.target_x_,self.target_y_, self.theta)
                            marker_text_target = "A({:.2f}, {:.2f})".format( self.target_x, self.target_y)
                            cv2.putText(color_image, marker_text_target, target_org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            self.target = [self.target_x,self.target_y]


            cv2.imshow("ArUco Marker Detection", color_image)
            
            self.start_pub.publish(self.start)
            self.target_pub.publish(self.target)
            self.rate.sleep()

             rospy.loginfo("Publishing CameraInfo")
        finally:
            self.pipeline.stop()

if __name__ == '__main__':
    try:
        coords_publisher = CoordsPublisher()
        coords_publisher.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
