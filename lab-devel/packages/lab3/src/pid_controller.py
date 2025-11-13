#!/usr/bin/env python3
import rospy
import time
import os
from duckietown_msgs.msg import LanePose, Twist2DStamped

class PIDLaneController:
    def __init__(self):
        rospy.set_param("controller_ready", "true")

        # Retrieve PID parameters
        self.kp = rospy.get_param("~kp", 3.0)
        self.ki = rospy.get_param("~ki", 0.0)
        self.kd = rospy.get_param("~kd", 0.5)
        self.v = rospy.get_param("~v", 0.0)

        # Internal PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        # Subscribers & Publishers
        veh_name = os.environ['VEHICLE_NAME']
        rospy.Subscriber(f"/{veh_name}/lane_filter_node/lane_pose", LanePose, self.lane_pose_cb)
        self.pub_cmd = rospy.Publisher(f"/{veh_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)


    def lane_pose_cb(self, msg):
        # Retrieve PID parameters
        self.kp = rospy.get_param("~kp", 3.0)
        self.ki = rospy.get_param("~ki", 0.0)
        self.kd = rospy.get_param("~kd", 0.5)
        self.v = rospy.get_param("~v", 0.0)

        phi = msg.phi  # Current orientation angle (radians)
        desired_phi = 0.0 # Reference orientation

        # Compute error and time step
        error = desired_phi - phi
        now = time.time()
        dt = 0.0 if self.prev_time is None else now - self.prev_time
        self.prev_time = now

        # PID terms
        self.integral += error * dt
        derivative = 0.0 if dt == 0 else (error - self.prev_error) / dt

        # PID output
        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Prevent extreme angular speeds
        control_output = max(min(control_output, 8.0), -8.0)

        # Build Twist2DStamped message
        car_cmd = Twist2DStamped()
        car_cmd.v = self.v 
        car_cmd.omega = control_output

        # Publish command
        self.pub_cmd.publish(car_cmd)

        # Update previous error
        self.prev_error = error


if __name__ == "__main__":
    rospy.init_node("pid_lane_controller", anonymous=True)
    PIDLaneController()
    rospy.spin()
