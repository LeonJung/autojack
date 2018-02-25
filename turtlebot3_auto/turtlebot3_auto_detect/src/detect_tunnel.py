#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8, String
from enum import Enum
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import math
import tf

class DetectLevel():
    def __init__(self):

        self.showing_final_image = "on"
        self.sub_image_original_type = "raw" # you can choose image type "compressed", "raw"
        self.showing_images = "off" # you can choose showing images or not by "on", "off"

        # if self.sub_image_original_type == "compressed":
        #     # subscribes compressed image
        #     self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.callback, queue_size = 1)
        # elif self.sub_image_original_type == "raw":
        #     # subscribes raw image
        #     self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.callback, queue_size = 1)


        # # subscribes state : white line reliability
        # self.sub_white_line_reliability = rospy.Subscriber('/detect/white_line_reliability', UInt8, self.cbWhiteLineReliable, queue_size=1)
 
        self.sub_level_crossing_order = rospy.Subscriber('/detect/level_crossing_order', UInt8, self.cbLevelCrossingOrder, queue_size=1)

        # self.sub_obstacle = rospy.Subscriber('/detect/obstacle', UInt8, self.cbObstacleDetected, queue_size=1)

        # self.sub_level_crossing_finished = rospy.Subscriber('/control/level_crossing_finished', UInt8, self.cbLevelCrossingFinished, queue_size = 1)

        self.pub_level_crossing_return = rospy.Publisher('/detect/level_crossing_stamped', UInt8, queue_size=1)

        # self.pub_parking_start = rospy.Publisher('/control/parking_start', UInt8, queue_size = 1)

        self.StepOfLevelCrossing = Enum('StepOfLevelCrossing', 'searching_stop_sign searching_level watching_level stop pass_level')


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

        self.cvBridge = CvBridge()
        self.cv_image = None

        self.is_parking_finished = False

        # rospy.sleep(1)

        # loop_rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.fnFindDotLine()

        #     loop_rate.sleep()


    def cbGetImage(self, image_msg):
        if self.sub_image_original_type == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        else:
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "mono8")

        # if self.sub_image_original_type == "compressed":
        #     np_arr = np.fromstring(image_msg.data, np.uint8)
        #     self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # else:
        #     self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        # cv2.imshow('self.cv_image', self.cv_image), cv2.waitKey(1)

    def cbLevelCrossingOrder(self, order):
        pub_level_crossing_return = UInt8()

        if order.data == self.StepOfLevelCrossing.searching_stop_sign.value:
            rospy.loginfo("Now lane_following")

            pub_level_crossing_return.data = self.StepOfLevelCrossing.searching_stop_sign.value
                            
                                
        elif order.data == self.StepOfLevelCrossing.searching_level.value:
            rospy.loginfo("Now searching_level")

            # while True:
            #     # rospy.loginfo("white_line_reliability : %d", self.white_line_reliability)
            #     # if self.white_line_reliability < 80:
            #     #     break
            #     if self.fnFindDotLine():
            #         break
            #     else:
            #         pass

            pub_level_crossing_return.data = self.StepOfLevelCrossing.searching_level.value


        elif order.data == self.StepOfLevelCrossing.watching_level.value:
            rospy.loginfo("Now watching_level")

            # while True:
            #     if self.is_obstacle_detected == False:
            #         rospy.loginfo("parking lot is clear")
            #         break
            #     else:
            #         rospy.loginfo("right side is full")

            pub_level_crossing_return.data = self.StepOfLevelCrossing.watching_level.value


        elif order.data == self.StepOfLevelCrossing.stop.value:
            rospy.loginfo("Now stop")

            # msg_parking_start = UInt8()
            # msg_parking_start.data = 1
            # self.pub_parking_start.publish(msg_parking_start)

            # # waiting for finishing parking
            # while 1:
            #     if self.is_parking_finished == True:
            #         break

            pub_level_crossing_return.data = self.StepOfLevelCrossing.stop.value

        elif order.data == self.StepOfLevelCrossing.pass_level.value:
            rospy.loginfo("Now pass_level")

            # while True:
            #     if self.is_obstacle_detected == False:
            #         rospy.loginfo("parking lot is clear")
            #         break
            #     else:
            #         rospy.loginfo("right side is full")

            pub_level_crossing_return.data = self.StepOfLevelCrossing.pass_level.value

        self.pub_level_crossing_return.publish(pub_level_crossing_return)


    def fnParkingLotOrder(self):
        while True:
            self.fnFindDotLine()

    def cbParkingFinished(self, parking_finished_msg):
        self.is_parking_finished = True

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_parking')
    node = DetectLevel()
    node.main()
