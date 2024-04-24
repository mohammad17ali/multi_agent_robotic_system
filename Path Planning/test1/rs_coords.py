#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RealSenseCameraNode:
    def __init__(self):
        rospy.init_node('realsense_camera_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        self.camera = cv2.VideoCapture(0)  # Change index if necessary

    def capture_and_publish(self):
        rate = rospy.Rate(10)  # Adjust as necessary
        while not rospy.is_shutdown():
            ret, frame = self.camera.read()
            if ret:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(img_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        camera_node = RealSenseCameraNode()
        camera_node.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
