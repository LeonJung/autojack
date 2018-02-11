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

class DetectParking():
    def __init__(self):
        # subscribes state : white line reliability
        self.sub_white_line_reliability = rospy.Subscriber('/detect/white_line_reliability', UInt8, self.cbWhiteLineReliable, queue_size=1)
 
        self.sub_parking_lot_order = rospy.Subscriber('/detect/parking_lot_order', UInt8, self.cbParkingLotOrder, queue_size=1)

        self.sub_obstacle = rospy.Subscriber('/detect/obstacle', UInt8, self.cbObstacleDetected, queue_size=1)

        self.pub_parking_lot_return = rospy.Publisher('/detect/parking_lot_stamped', UInt8, queue_size=1)



        self.StepOfParkingLot = Enum('StepOfParkingLot', 'searching_parking_sign searching_parking_point_line searching_nonreserved_parking_area parking')


        # Two variables below will be used when there is no lane
        # self.lane_position : -1 means lane is in left and 1 is opposite
        # When there is no lane detected, turtlebot will turn to the direction where lane was existed in the last cycle
        self.lane_position = 0

        self.position_now = None
        self.theta_now = None

        self.position_parking_detected = None

        self.position_turning_point = None
        self.theta_turning_point = None

        self.position_parking = None

        self.is_obstacle_detected = False
        self.white_line_reliability = 100


    def cbParkingLotOrder(self, order):
        msg_pub_parking_lot_return = UInt8()

        if order.data == self.StepOfParkingLot.searching_parking_sign.value:
            rospy.loginfo("Now lane_following")

            msg_pub_parking_lot_return.data = self.StepOfParkingLot.searching_parking_sign.value
                            
                                
        elif order.data == self.StepOfParkingLot.searching_parking_point_line.value:
            rospy.loginfo("Now searching_parking_point_line")

            while True:
                rospy.loginfo("white_line_reliability : %d", self.white_line_reliability)
                if self.white_line_reliability < 80:
                    break

            msg_pub_parking_lot_return.data = self.StepOfParkingLot.searching_parking_point_line.value


        elif order.data == self.StepOfParkingLot.searching_nonreserved_parking_area.value:
            rospy.loginfo("Now searching_nonreserved_parking_area")

            while True:
                if self.is_obstacle_detected == False:
                    rospy.loginfo("parking lot is clear")
                    break

            msg_pub_parking_lot_return.data = self.StepOfParkingLot.searching_nonreserved_parking_area.value


        elif order.data == self.StepOfParkingLot.parking.value:
            rospy.loginfo("Now parking")



            msg_pub_parking_lot_return.data = self.StepOfParkingLot.parking.value

        self.pub_parking_lot_return.publish(msg_pub_parking_lot_return)

    def cbWhiteLineReliable(self, white_line_reliability):
        self.white_line_reliability = white_line_reliability.data

    def cbObstacleDetected(self, scan):
        # angle_scan = 15
        # scan_start = 270 - angle_scan
        # scan_end = 270 + angle_scan
        # threshold_distance = 0.3
        # obstacle_existence = 'no'
        # obstacle_count = 0



        angle_scan = 45
        scan_start = 270 - angle_scan
        scan_end = 270 + angle_scan
        threshold_distance = 0.5
        obstacle_existence = False
        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                obstacle_existence = True




        print obstacle_existence

        self.is_obstacle_detected = obstacle_existence



    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_parking')
    node = DetectParking()
    node.main()
