#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from duckietown_msgs.msg import Segment, SegmentList
from geometry_msgs.msg import Point


class RedStopLineDetector:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.Subscriber(
            "/ee483mm05/camera_node/image/compressed",
            CompressedImage,
            self.process,
            queue_size=1,
            buff_size=10000000
        )

        # like Lab 2
        self.pub_filtered = rospy.Publisher("filtered_image", Image, queue_size=10)
        self.pub_edges = rospy.Publisher("edges", Image, queue_size=10)
        self.pub_output = rospy.Publisher("red_lines", Image, queue_size=10)

        # Pixel-space red segments for ground projection
        # self.pub_segments = rospy.Publisher(
        #     "~lineseglist_out", SegmentList, queue_size=1
        # )

    def process(self, msg):
        # Convert compressed --> cv2 BGR
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Crop bottom 60% 
        h, w = frame.shape[:2]
        cropped = frame[int(h * 0.4):h, 0:w]

        # Convert to HSV
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # Red color masks (two ranges)
        lower_red1 = np.array([0, 120, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 80])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Apply mask to image
        filtered = cv2.bitwise_and(cropped, cropped, mask=mask)

        # Edge Detection 
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Hough Line Detection 
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180,
            threshold=20,
            minLineLength=20,
            maxLineGap=10
        )

        # Copy cropped image to draw line overlay
        output = np.copy(cropped)

        # SegmentList message
        seg_list = SegmentList()
        seg_list.header.stamp = rospy.Time.now()

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Draw for debug image
                cv2.line(output, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)
                cv2.circle(output, (x1, y1), 3, (0, 255, 0), -1)
                cv2.circle(output, (x2, y2), 3, (0, 255, 0), -1)

                # Build Segment msg for dt-core pipeline
                seg = Segment()
                seg.color = Segment.RED

                p1 = Point()
                p1.x = x1 / float(w)
                p1.y = y1 / float(h)

                p2 = Point()
                p2.x = x2 / float(w)
                p2.y = y2 / float(h)

                seg.pixels_normalized[0] = p1
                seg.pixels_normalized[1] = p2

                seg_list.segments.append(seg)

        # Publish segment list
        self.pub_segments.publish(seg_list)

        # Convert and publish debug images
        ros_filtered = self.bridge.cv2_to_imgmsg(filtered, "bgr8")
        ros_edges = self.bridge.cv2_to_imgmsg(edges, "mono8")
        ros_output = self.bridge.cv2_to_imgmsg(output, "bgr8")

        self.pub_filtered.publish(ros_filtered)
        self.pub_edges.publish(ros_edges)
        self.pub_output.publish(ros_output)


if __name__ == "__main__":
    rospy.init_node("red_stopline_detector")
    node = RedStopLineDetector()
    rospy.spin()
