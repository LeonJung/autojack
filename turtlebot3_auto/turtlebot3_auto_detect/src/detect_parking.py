#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8, Float64, String
from enum import Enum
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import math
import tf

class DetectParking():
    def __init__(self):
        self.sub_image_type = "raw"         # "compressed" / "raw"
        self.pub_image_type = "raw"         # "compressed" / "raw"

        self.showing_images = "off" # you can choose showing images or not by "on", "off"

        # subscribes state : white line reliability
        self.sub_white_line_reliability = rospy.Subscriber('/detect/white_line_reliability', UInt8, self.cbWhiteLineReliable, queue_size=1)
 
        self.sub_parking_lot_order = rospy.Subscriber('/detect/parking_lot_order', UInt8, self.cbParkingLotOrder, queue_size=1)

        self.sub_scan_obstacle = rospy.Subscriber('/detect/scan', LaserScan, self.cbScanObstacle, queue_size=1)

        self.sub_parking_finished = rospy.Subscriber('/control/parking_finished', UInt8, self.cbParkingFinished, queue_size = 1)

        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.cbGetImage, queue_size = 1)
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.cbGetImage, queue_size = 1)

        if self.pub_image_type == "compressed":
            # publishes compensated image in compressed type 
            self.pub_image_parking = rospy.Publisher('/detect/image_output/compressed', CompressedImage, queue_size = 1)
        elif self.pub_image_type == "raw":
            # publishes compensated image in raw type
            self.pub_image_parking = rospy.Publisher('/detect/image_output', Image, queue_size = 1)

        self.pub_parking_lot_return = rospy.Publisher('/detect/parking_lot_stamped', UInt8, queue_size=1)

        self.pub_parking_start = rospy.Publisher('/control/parking_start', UInt8, queue_size = 1)

        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)

        self.StepOfParkingLot = Enum('StepOfParkingLot', 'searching_parking_sign searching_parking_point_line searching_nonreserved_parking_area parking')

        self.is_obstacle_detected = True
        self.white_line_reliability = 100

        self.cvBridge = CvBridge()
        self.cv_image = None

        self.is_now_parking = False
        self.is_parking_finished = False

        self.blink_trigger = 1
        self.blink_count = 0

        self.img1 = cv2.imread('/home/leon/Desktop/parking_unavailable.png', -1)
        self.img2 = cv2.imread('/home/leon/Desktop/parking_available.png', -1)



    def cbGetImage(self, image_msg):
        if self.sub_image_type == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        if self.is_now_parking == True:
            self.blink_count += 1
            if self.blink_count % 4 == 0:
                self.blink_trigger = 1 - self.blink_trigger
                self.blink_count = 0

            if self.is_obstacle_detected == True:
                if self.blink_trigger == 1:
                    x_offset = 200
                    y_offset = 50

                    y1, y2 = y_offset, y_offset + self.img1.shape[0]
                    x1, x2 = x_offset, x_offset + self.img1.shape[1]

                    alpha_s = self.img1[:, :, 3] / 255.0
                    alpha_l = 1.0 - alpha_s

                    for c in range(0, 3):
                        self.cv_image[y1:y2, x1:x2, c] = (alpha_s * self.img1[:, :, c] +
                                                alpha_l * self.cv_image[y1:y2, x1:x2, c])

            else:
                if self.blink_trigger == 1:
                    x_offset = 200
                    y_offset = 50

                    y1, y2 = y_offset, y_offset + self.img2.shape[0]
                    x1, x2 = x_offset, x_offset + self.img2.shape[1]

                    alpha_s = self.img2[:, :, 3] / 255.0
                    alpha_l = 1.0 - alpha_s

                    for c in range(0, 3):
                        self.cv_image[y1:y2, x1:x2, c] = (alpha_s * self.img2[:, :, c] +
                                                alpha_l * self.cv_image[y1:y2, x1:x2, c])


        if self.pub_image_type == "compressed":
            # publishes compensated image in compressed type
            self.pub_image_parking.publish(self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes compensated image in raw type
            self.pub_image_parking.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))

    def cbParkingLotOrder(self, order):
        msg_pub_parking_lot_return = UInt8()

        if order.data == self.StepOfParkingLot.searching_parking_sign.value:
            rospy.loginfo("Now lane_following")

            msg_pub_parking_lot_return.data = self.StepOfParkingLot.searching_parking_sign.value
                            
                                
        elif order.data == self.StepOfParkingLot.searching_parking_point_line.value:
            rospy.loginfo("Now searching_parking_point_line")

            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.06
            self.pub_max_vel.publish(msg_pub_max_vel)

            while True:
                if self.fnFindDotLine():
                    break
                else:
                    pass

            msg_pub_parking_lot_return.data = self.StepOfParkingLot.searching_parking_point_line.value


        elif order.data == self.StepOfParkingLot.searching_nonreserved_parking_area.value:
            rospy.loginfo("Now searching_nonreserved_parking_area")

            self.is_now_parking = True

            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.10
            self.pub_max_vel.publish(msg_pub_max_vel)

            while True:
                if self.is_obstacle_detected == False:
                    rospy.loginfo("parking lot is clear")
                    break
                else:
                    rospy.loginfo("right side is full")

            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.0
            self.pub_max_vel.publish(msg_pub_max_vel)


            msg_pub_parking_lot_return.data = self.StepOfParkingLot.searching_nonreserved_parking_area.value


        elif order.data == self.StepOfParkingLot.parking.value:
            rospy.loginfo("Now parking")

            msg_parking_start = UInt8()
            msg_parking_start.data = 1
            self.pub_parking_start.publish(msg_parking_start)

            # waiting for finishing parking
            while 1:
                if self.is_parking_finished == True:
                    break

            self.is_now_parking = False

            msg_pub_parking_lot_return.data = self.StepOfParkingLot.parking.value

        self.pub_parking_lot_return.publish(msg_pub_parking_lot_return)

    def cbParkingFinished(self, parking_finished_msg):
        self.is_parking_finished = True

    def cbWhiteLineReliable(self, white_line_reliability):
        self.white_line_reliability = white_line_reliability.data

    def cbScanObstacle(self, scan):
        # angle_scan = 15
        # scan_start = 270 - angle_scan
        # scan_end = 270 + angle_scan
        # threshold_distance = 0.3
        # is_obstacle_detected = 'no'
        # obstacle_count = 0

        angle_scan = 60#45
        scan_start = 270 - angle_scan
        scan_end = 270 + angle_scan
        threshold_distance = 0.5
        is_obstacle_detected = False

        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                is_obstacle_detected = True

        self.is_obstacle_detected = is_obstacle_detected

    def fnFindDotLine(self):
        self.cv_bitn_img = cv2.bitwise_not(self.cv_image)
        params = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 254

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 5000
        params.maxArea = 6000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.5

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.8

        det = cv2.SimpleBlobDetector_create(params)
        keypts = det.detect(self.cv_bitn_img)
        frame = cv2.drawKeypoints(self.cv_bitn_img, keypts, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # showing binary and original images
        if self.showing_images == 'on':
            # cv2.imshow('cv_image', self.cv_image), cv2.waitKey(1)
            cv2.imshow('frame', frame), cv2.waitKey(1)

        count = 0
        for i in range(len(keypts)):
            x = int(keypts[i].pt[0])
            y = int(keypts[i].pt[1])

            if x > 500:
            # if y > 100 and y < 500 :
                count += 1

        if count >= 1:
            return True
        else:
            return False

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_parking')
    node = DetectParking()
    node.main()
