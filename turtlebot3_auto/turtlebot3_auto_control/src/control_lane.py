#!/usr/bin/env python

#Special thanks to chaosmail (https://gist.github.com/chaosmail/8372717#file-python-pid_controller)
#(https://www.robotshop.com/letsmakerobots/files/PID_Line_Following_Tutorials.pdf)

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt

def callback(x):    
    pass

class ControlLane():
    def __init__(self):
        self.showing_images = "off" # you can choose showing images or not by "on", "off"
        self.selecting_sub_image = "compressed" # you can choose image type "compressed", "raw"
        self.selecting_pub_image = "compressed" # you can choose image type "compressed", "raw"

        self.sub_lane = rospy.Subscriber('/control/lane', Float64, self.callback, queue_size = 1)

        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)

        self.lastError = 0


        rospy.on_shutdown(self.fnShutDown)

    def callback(self, desired_center):
        center = desired_center.data

        error = center - 500

        MAX_VEL = 0.19

        Kp = 0.0035#0.0035
        Kd = 0.007#0.007

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error

        # print(center)

        # print(error)
        # print(self.lastError)
        # print(angular_z)
        # print((1 - error / 500))       

        twist = Twist()
        twist.linear.x = MAX_VEL * ((1 - abs(error) / 500) ** 2) 
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)


    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()



if __name__ == '__main__':
    rospy.init_node('control_lane')
    node = ControlLane()
    node.main()
