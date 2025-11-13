#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import LanePose

class MovingAverageFilter:
    def __init__(self):
        # Vehicle name from environment
        veh_name = os.environ['VEHICLE_NAME']
        self.window_size = 5
        self.values = []

        # Topics
        sub_topic = f"/{veh_name}/lane_filter_node/lane_pose"
        pub_topic = f"/{veh_name}/lane_filter_node/lane_pose_filtered"

        # Subscriber and Publisher
        rospy.Subscriber(sub_topic, LanePose, self.pose_callback)
        self.pub = rospy.Publisher(pub_topic, LanePose, queue_size=10)

    def pose_callback(self, msg):
        # Store recent phi values
        self.values.append(msg.phi)
        if len(self.values) > self.window_size:
            self.values.pop(0)

        # Compute moving average
        phi_avg = sum(self.values) / len(self.values)

        # Create and publish filtered message
        filtered_pose = LanePose()
        filtered_pose.header = msg.header
        filtered_pose.d = msg.d
        filtered_pose.phi = phi_avg
        filtered_pose.in_lane = msg.in_lane

        self.pub.publish(filtered_pose)

if __name__ == "__main__":
    rospy.init_node("lane_filter_moving_average", anonymous=True)
    MovingAverageFilter()
    rospy.spin()
