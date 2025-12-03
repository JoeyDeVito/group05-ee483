#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from std_msgs.msg import Bool

class BehaviorManager:
    def __init__(self):
        veh = os.environ['VEHICLE_NAME']

        # STATE MACHINE
        self.state = "LANE_FOLLOWING"
        self.stop_time = None
        self.stop_triggered_once = False

        # SUBSCRIBERS
        rospy.Subscriber(f"/{veh}/stopline_detected", Bool, self.cb_stop)
        rospy.Subscriber(f"/{veh}/pid_lane_controller/cmd", Twist2DStamped, self.cb_pid)

        # PUBLISHER
        self.pub_wheels = rospy.Publisher(
            f"/{veh}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=10
        )

        # DATA
        self.stop_detected = False

    def cb_stop(self, msg):
        self.stop_detected = msg.data

    def cb_pid(self, pid_cmd):
        wheels = WheelsCmdStamped()

        # ------------------------------
        # STATE: NORMAL LANE FOLLOWING
        # ------------------------------
        if self.state == "LANE_FOLLOWING":
            # FIRST TIME redline detected
            if self.stop_detected and not self.stop_triggered_once:
                self.stop_triggered_once = True
                self.state = "STOP_AT_LINE"
                self.stop_time = rospy.Time.now()

                wheels.vel_left = 0
                wheels.vel_right = 0
            else:
                # convert Twist to wheel commands
                wheels.vel_left = pid_cmd.v + pid_cmd.omega
                wheels.vel_right = pid_cmd.v - pid_cmd.omega

        # ------------------------------
        # STATE: FULL STOP FOR 2 SECONDS
        # ------------------------------
        elif self.state == "STOP_AT_LINE":
            wheels.vel_left = 0
            wheels.vel_right = 0

            if (rospy.Time.now() - self.stop_time).to_sec() > 2.0:
                self.state = "FORWARD"
                self.stop_time = rospy.Time.now()

        # ------------------------------
        # STATE: MOVE FORWARD BEFORE TURN
        # ------------------------------
        elif self.state == "FORWARD":
            wheels.vel_left = 0.25
            wheels.vel_right = 0.25

            if (rospy.Time.now() - self.stop_time).to_sec() > 1:
                self.state = "LANE_FOLLOWING" #Back to pid
                self.stop_time = rospy.Time.now()


        # Output wheels
        self.pub_wheels.publish(wheels)

if __name__ == "__main__":
    rospy.init_node("behavior_manager")
    BehaviorManager()
    rospy.spin()
