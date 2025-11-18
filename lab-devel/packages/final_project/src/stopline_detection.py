#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from duckietown_msgs.msg import Segment, SegmentList
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class RedSegmentDetector:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribe to raw camera feed (compressed)
        rospy.Subscriber("/ee483mm05/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=10000000)

        # Publish SegmentList (pixel-space) for ground projection
        # self.pub_segments = rospy.Publisher("~lineseglist_out", SegmentList, queue_size=1)
        self.pub = rospy.Publisher("filtered_image", Image, queue_size=10)

    def callback(self, msg):
        # Convert camera image
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        height, width = frame.shape[:2]

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 80])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([160, 120, 80])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Edge detection
        edges = cv2.Canny(mask, 50, 150)

        # Hough line detection
        lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi / 180,threshold=30,minLineLength=20,maxLineGap=10
        )

        # Build SegmentList message
        seg_list = SegmentList()
        seg_list.header.stamp = rospy.Time.now()

        if lines is not None:
            for l in lines:
                x1, y1, x2, y2 = l[0]

                seg = Segment()
                seg.color = Segment.RED

                # Normalize pixel coords to [0,1]
                p1 = Point()
                p1.x = float(x1) / width
                p1.y = float(y1) / height

                p2 = Point()
                p2.x = float(x2) / width
                p2.y = float(y2) / height

                # Assign to Segment message
                seg.pixels_normalized[0] = p1
                seg.pixels_normalized[1] = p2

                seg_list.segments.append(seg)

        # Publish to ground projection input
        self.pub_segments.publish(seg_list)


if __name__ == "__main__":
    rospy.init_node("red_segment_detector", anonymous=False)
    node = RedSegmentDetector()
    rospy.spin()
