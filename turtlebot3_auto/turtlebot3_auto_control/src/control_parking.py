#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8, String
from enum import Enum
from geometry_msgs.msg import Twist
import math
import tf

class ControlParking():
    def __init__(self):
        aaa = 0

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_parking')
    node = ControlParking()
    node.main()
