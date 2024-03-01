import cv2
import cv2.aruco as aruco
import pyrealsense2
import matplotlib.pyplot as plt
import numpy as np
from functions import *
from realsense_depth import *


aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()


dc = DepthCamera()

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
aruco_params = cv2.aruco.DetectorParameters_create()

try:               
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
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
                        x1_ = depth_point[0]
                        y1_ = depth_point[1]
                        z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                        org_a = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)


                elif ids[i] == 2:
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
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


                x1_, y1_, x2_,y2_ = round(x1_,2),round(y1_,2), round(x2_,2), round(y2_,2)
                calib_coords = [x1_, y1_, x2_,y2_]
                theta = angle(y1_,y2_,x1_,x2_)

                if (1 in ids) and (2 in ids):
                    x1,y1 = coordinates(calib_coords,x1_,y1_,theta)
                    x2,y2 = coordinates(calib_coords,x2_,y2_,theta)
                    marker_text_a = "A({:.2f}, {:.2f})".format( x1, y1)
                    marker_text_b = "B({:.2f}, {:.2f})".format(x2, y2)
                    cv2.line(color_image, p1, p2,(255,0,0), 1)
                    #cv2.putText(color_image, marker_text_a, org_a, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    #cv2.putText(color_image, marker_text_b, org_b, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                else:
                    break

        if ids is not None:
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)

                center = np.mean(corners[i][0], axis=0).astype(int)
                depth_value = depth_frame.get_distance(center[0], center[1])

                if depth_value > 0:
                    depth_point = rs.rs2_deproject_pixel_to_point(
                        depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)

                    marker_text = "Marker ID: {} | Coordinates: {:.2f}, {:.2f}, {:.2f}".format(ids[i][0], depth_point[0], depth_point[1], depth_point[2])
                    
                    marker_x_ = depth_point[0]
                    marker_y_ = depth_point[1]
                    marker_z = depth_point[2]
                    
                    #using the calibrated values
                    marker_x,marker_y = coordinates(calib_coords,marker_x_,marker_y_,theta)
                    marker_text = "ID: {} ({:.2f}, {:.2f})".format(ids[i][0], marker_x, marker_y)

                    # Convert coordinates to integers before passing to putText
                    org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)

                    cv2.putText(color_image, marker_text, org,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    cv2.aruco.drawDetectedMarkers(color_image, corners)

        cv2.imshow("ArUco Marker Detection", color_image)

        # Exit the program when the 'Esc' key is pressed
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

plt.figure(figsize = (10,10))
plt.imshow(color_image)
