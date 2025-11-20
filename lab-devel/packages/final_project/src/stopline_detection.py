#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import SegmentList, Segment
from std_msgs.msg import Float32, Bool

class StoplineGroundDetector:
    def __init__(self):
        rospy.loginfo("Starting Stopline Ground Detector Node")

        veh = os.environ['VEHICLE_NAME']

        # Subscribe to ground-projected segment list
        self.sub = rospy.Subscriber(f"/{veh}/ground_projection_node/lineseglist_out",SegmentList,self.cb_segments,queue_size=1)

        # Distance to stop line (meters)
        self.pub_dist = rospy.Publisher("stopline_distance", Float32, queue_size=10)

        # Boolean flag: True = STOP NOW
        self.pub_stop = rospy.Publisher("stopline_detected", Bool, queue_size=10)

        # distance threshold in meters (tune)
        self.stop_threshold = 0.22

    def cb_segments(self, msg):
        red_x_values = []

        for seg in msg.segments:
            if seg.color != Segment.RED:
                continue

            # Extract ground-plane x-values (front-back distance)
            x1 = seg.points[0].x
            x2 = seg.points[1].x

            # Only consider points IN FRONT of the robot (positive x)
            for x in [x1, x2]:
                if x > 0.0:
                    red_x_values.append(x)

        if len(red_x_values) == 0:
            self.pub_stop.publish(False)
            self.pub_dist.publish(Float32(999.0))
            return

        # Smallest positive x = closest stop line point
        dist = min(red_x_values)

        self.pub_dist.publish(Float32(dist))

        # Check if within stopping range
        should_stop = (dist < self.stop_threshold)
        self.pub_stop.publish(Bool(should_stop))

        rospy.loginfo(f"Stopline distance: {dist:.3f} m   STOP={should_stop}")


if __name__ == "__main__":
    rospy.init_node("stopline_ground_detector")
    StoplineGroundDetector()
    rospy.spin()
