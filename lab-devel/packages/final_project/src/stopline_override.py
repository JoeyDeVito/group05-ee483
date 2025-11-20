#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Bool

class StoplineOverride:
    def __init__(self):
        self.stop_detected = False

        # Subscribe to PID controller outputs 
        rospy.Subscriber("pid_controller_cmd",
                         Twist2DStamped,
                         self.cmd_cb)

        # Subscribe to stop signal 
        rospy.Subscriber("stopline_detected",
                         Bool,
                         self.stop_cb)

        # Publish final motor command 
        veh = os.environ['VEHICLE_NAME']
        self.pub_cmd = rospy.Publisher(
            f"/{veh}/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=10
        )

    def stop_cb(self, msg):
        self.stop_detected = msg.data

    def cmd_cb(self, cmd):
        final = Twist2DStamped()
        final.header.stamp = rospy.Time.now()

        if self.stop_detected:
            final.v = 0.0
            final.omega = 0.0
        else:
            final.v = cmd.v
            final.omega = cmd.omega

        self.pub_cmd.publish(final)

if __name__ == "__main__":
    rospy.init_node("stopline_override")
    StoplineOverride()
    rospy.spin()
