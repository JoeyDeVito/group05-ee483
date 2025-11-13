#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped # Import the message for the wheel comm

#    <node pkg="final_project" name="driver_node" type="right_turn.py" output="screen"/>

class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']
        self.pub = rospy.Publisher('/ee483mm05/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.msg = WheelsCmdStamped()


    def drive(self):
        self.straight(0.25, 0.20)
        rospy.sleep(1.5)
        self.stop()

        rospy.sleep(5)

        self.right()
        rospy.sleep(0.8)
        self.stop()

        self.straight(0.25, 0.20)
        rospy.sleep(2.0)
        self.stop()


    def straight(self, left, right):
        print("Moving straight")
        self.msg.vel_left = left
        self.msg.vel_right = right
        self.pub.publish(self.msg)



    def right(self):
        print("Turning right")
        self.msg.vel_right = -0.3
        self.msg.vel_left = 0.3
        self.pub.publish(self.msg)

    def stop(self):
        print("Stopping")
        self.msg.vel_left = 0
        self.msg.vel_right = 0
        self.pub.publish(self.msg)



if __name__ == "__main__": ## The main function which will be called when your python sc
# Initialize the node
    try:
        rospy.init_node('driving')
        drive = Driver() # Create obj of the Driver class
        rospy.sleep(3) # Delay to wait enough time for the code to run
        # Keep the line above - you might be able to reduce the delay a bit,
        #while not rospy.is_shutdown(): # Run ros forever - you can change
            # this as well instead of running forever
        drive.drive() # calling your node function
    except rospy.ROSInterruptException:
        pass