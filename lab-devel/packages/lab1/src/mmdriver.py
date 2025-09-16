#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped # Import the message for the wheel comm

class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']
        # USING PARAMETER TO GET THE NAME OF THE VEHICLE
        # THIS WILL BE USEFUL TO SPECIFY THE NAME OF THE TOPIC
        # INITIALIZE YOUR VARIABLES HERE (SUBSCRIBERS OR PUBLISHERS)
        self.pub = rospy.Publisher('/ee483mm05/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.msg = WheelsCmdStamped()


    def drive(self): # CHANGE TO THE NAME OF YOUR FUNCTION
        # WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
        
        for i in range(4):
            self.straight()
            rospy.sleep(3)
            self.stop()

            rospy.sleep(3)

            self.right()
            rospy.sleep(0.8)
            self.stop()
            
            rospy.sleep(3)
            

        self.stop()


    def straight(self):
        print("Moving straight")
        self.msg.vel_left = 0.5
        self.msg.vel_right = 0.5
        self.pub.publish(self.msg)



    def right(self):
        print("Turning right")
        self.msg.vel_right = 0.5
        self.msg.vel_left = -0.5
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