#!/usr/bin/env python

#Special thanks to chaosmail (https://gist.github.com/chaosmail/8372717#file-python-pid_controller)

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt

def callback(x):    
    pass

class PathFollowing():
    def __init__(self):
        self.showing_images = "off" # you can choose showing images or not by "on", "off"
        self.selecting_sub_image = "compressed" # you can choose image type "compressed", "raw"
        self.selecting_pub_image = "compressed" # you can choose image type "compressed", "raw"

        self._sub = rospy.Subscriber('/lane/desired_center', Float64, self.callback, queue_size = 1)
        # self._sub = rospy.Subscriber('/lane/off_center', Float64, self.callback, queue_size = 1)

        self._pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        # There are 4 Publishers
        # pub1 : calibrated image as compressed image
        # pub2 : Bird's eye view image as compressed image
        # pub3 : calibrated image as raw image
        # pub4 : Bird's eye view image as raw image
        # if you do not want to publish some topic, delete pub definition in __init__ function and publishing process in callback function
        # if self.selecting_pub_image == "compressed":
        #     self._pub1 = rospy.Publisher('/image_lanes_compressed', CompressedImage, queue_size = 1)
        # elif self.selecting_pub_image == "raw":
        #     self._pub2 = rospy.Publisher('/image_lanes', Image, queue_size = 1)

        # self._pub3 = rospy.Publisher('/lane/radius', Float64, queue_size = 1)
        # self._pub4 = rospy.Publisher('/lane/off_center', Float64, queue_size = 1)


        self.counter = 0


    def callback(self, desired_center):
        center = desired_center.data

        if self.counter == 0:
            lastError = 0


        error = center - 450

        Kp = 0.001
        Kd = 0.0005

        angular_z = Kp * error + Kd * (error - lastError)
        lastError = error

        print(center)

        print(error)
        print(lastError)
        print(angular_z)

        # rightMotorSpeed = RIGHT_BASE_SPEED + lastError;
        # leftMotorSpeed = LEFT_BASE_SPEED - lastError;
  
#     if (rightMotorSpeed > RIGHT_MAX_SPEED ) rightMotorSpeed = RIGHT_MAX_SPEED; // prevent the motor from going beyond max speed
#   if (leftMotorSpeed > LEFT_MAX_SPEED ) leftMotorSpeed = LEFT_MAX_SPEED; // prevent the motor from going beyond max speed
#   if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
#   if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive


        # max_vel = 1.0
        # P = 20.0
        # center = P * center 

        

        twist = Twist()
        twist.linear.x = 0.04#0.030
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self._pub1.publish(twist)
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        # if self.counter % 3 != 0:
        #     self.counter += 1
        #     return

        # if self.selecting_sub_image == "compressed":
            #converting compressed image to opencv image
            # np_arr = np.fromstring(image_msg.data, np.uint8)


    # def pid_controller(y, yc, h=1, Ti=1, Td=1, Kp=1, u0=0, e0=0)
    #     """Calculate System Input using a PID Controller

    #     Arguments:
    #     y  .. Measured Output of the System
    #     yc .. Desired Output of the System
    #     h  .. Sampling Time
    #     Kp .. Controller Gain Constant
    #     Ti .. Controller Integration Constant
    #     Td .. Controller Derivation Constant
    #     u0 .. Initial state of the integrator
    #     e0 .. Initial error

    #     Make sure this function gets called every h seconds!
    #     """
        
    #     # Step variable
    #     k = 0

    #     # Initialization
    #     ui_prev = u0
    #     e_prev = e0

    #     while 1:

    #         # Error between the desired and actual output
    #         e = yc - y

    #         # Integration Input
    #         ui = ui_prev + 1.0 / Ti * h * e
    #         # Derivation Input
    #         ud = 1.0 / Td * (e - e_prev) / (float)h

    #         # Adjust previous values
    #         e_prev = e
    #         ui_prev = ui

    #         # Calculate input for the system
    #         u = Kp * (e + ui + ud)
            
    #         k += 1

    #         yield u

    def main(self):
        rospy.spin()



if __name__ == '__main__':
    rospy.init_node('path_following')
    node = PathFollowing()
    node.main()
