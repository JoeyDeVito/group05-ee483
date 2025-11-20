#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import SegmentList, Segment
from std_msgs.msg import Float32, Bool

class StoplineGroundDetector:
    def __init__(self):
        rospy.loginfo("Starting Stopline Ground Detector Node")

        veh = os.environ['VEHICLE_NAME']

        rospy.Subscriber(
            f"/{veh}/ground_projection_node/lineseglist_out",
            SegmentList,
            self.cb_segments,
            queue_size=1
        )

        self.pub_dist = rospy.Publisher("stopline_distance", Float32, queue_size=10)
        self.pub_stop = rospy.Publisher("stopline_detected", Bool, queue_size=10)

        self.stop_threshold = 0.16

    def cb_segments(self, msg):
        red_x = []
        for seg in msg.segments:
            if seg.color != Segment.RED:
                continue
            for x in [seg.points[0].x, seg.points[1].x]:
                if x > 0:
                    red_x.append(x)

        if not red_x:
            self.pub_stop.publish(False)
            self.pub_dist.publish(Float32(999.0))
            return

        dist = min(red_x)
        should_stop = dist < self.stop_threshold

        self.pub_dist.publish(Float32(dist))
        self.pub_stop.publish(should_stop)

        rospy.loginfo(f"[STOPLINE] distance={dist:.3f} m stop={should_stop}")

if __name__ == "__main__":
    rospy.init_node("stopline_ground_detector")
    StoplineGroundDetector()
    rospy.spin()
