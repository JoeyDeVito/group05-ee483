#!/usr/bin/env python3
import os
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class LaneDetector:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()

        rospy.Subscriber("/ee483mm05/camera_node/image/compressed", CompressedImage, self.detector, queue_size=1, buff_size=10000000)

        self.pub1 = rospy.Publisher("edge_image", Image, queue_size=10)
        self.pub2 = rospy.Publisher("filtered_image", Image, queue_size=10)

    def detector(self, msg):
        # convert to a ROS image using the bridge
        #cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)

        #Crop 70% of the image
        h,w = cv_img.shape[:2]
        cropped = cv_img[int(h*0.4):h, 0:w]

        #Convert to HSV 
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        #White filter
        #lower_white = np.array([80,0,154])
        #upper_white = np.array([180,65,255])
        mask_white = cv2.inRange(hsv, (30,0,129), (180,255,255))

        #Yellow filter
        lower_yllw = np.array([18,80,80])
        upper_yllw = np.array([35,255,255])
        mask_yllw = cv2.inRange(hsv, lower_yllw, upper_yllw)

        #Combien the masks
        mask = cv2.bitwise_or(mask_white, mask_yllw)

        #Apply mask
        lane_img = cv2.bitwise_and(cropped, cropped, mask=mask)

        #Hough Transformation
        gray = cv2.cvtColor(lane_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 100, 200)

        #Detect lines using Hough Transformation
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 15, minLineLength=30, maxLineGap=10)

        #Draw detected lines
        output = np.copy(cropped)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1, y1), (x2, y2), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (x1, y1), 2, (0,255,0), -1)
                cv2.circle(output, (x2, y2), 2, (0,255,0), -1)

        #Convert new image to ROS to send
        ros_lane = self.bridge.cv2_to_imgmsg(output, "bgr8")

        #Filtering white and yellow
        ros_filter = self.bridge.cv2_to_imgmsg(lane_img, "bgr8")

        self.pub1.publish(ros_lane)
        self.pub2.publish(ros_filter)

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("lane_detector", anonymous=True)
    lane_detect = LaneDetector()
    rospy.spin()