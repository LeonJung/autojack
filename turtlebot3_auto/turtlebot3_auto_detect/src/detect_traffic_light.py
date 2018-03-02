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

def callback(x):
    pass

def fnCalcDistanceDot2Line(a, b, c, x0, y0):
    distance = abs(x0*a + y0*b + c)/math.sqrt(a*a + b*b)
    return distance

def fnCalcDistanceDot2Dot(x1, y1, x2, y2):
    distance = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
    return distance

def fnArrangeIndexOfPoint(arr):
    new_arr = arr[:]
    arr_idx = [0] * len(arr)
    for i in range(len(arr)):
        arr_idx[i] = i

    for i in range(len(arr)):
        for j in range(i+1, len(arr)):
            if arr[i] < arr[j]:
                buffer = arr_idx[j]
                arr_idx[j] = arr_idx[i]
                arr_idx[i] = buffer
                buffer = new_arr[j]
                new_arr[j] = new_arr[i]
                new_arr[i] = buffer
    return arr_idx

def fnCheckLinearity(point1, point2, point3):
    threshold_linearity = 50
    x1, y1 = point1
    x2, y2 = point3
    if x2-x1 != 0:
        a = (y2-y1)/(x2-x1)
    else:
        a = 1000
    b = -1
    c = y1 - a*x1
    err = fnCalcDistanceDot2Line(a, b, c, point2[0], point2[1])

    if err < threshold_linearity:
        return True
    else:
        return False

def fnCheckDistanceIsEqual(point1, point2, point3):

    threshold_distance_equality = 3
    distance1 = fnCalcDistanceDot2Dot(point1[0], point1[1], point2[0], point2[1])
    distance2 = fnCalcDistanceDot2Dot(point2[0], point2[1], point3[0], point3[1])
    std = np.std([distance1, distance2])

    if std < threshold_distance_equality:
        return True
    else:
        return False

class DetectTrafficLight():
    def __init__(self):
        self.showing_final_image = "off"
        self.showing_trackbar = "off"
        self.sub_image_original_type = "compressed" # you can choose image type "compressed", "raw"
        self.showing_images = "off" # you can choose showing images or not by "on", "off"

        if self.sub_image_original_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.cbGetImage, queue_size = 1)
        elif self.sub_image_original_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.cbGetImage, queue_size = 1)
 
        # self.sub_traffic_light_order = rospy.Subscriber('/detect/traffic_light_order', UInt8, self.cbTrafficLightOrder, queue_size=1)

        self.sub_traffic_light_finished = rospy.Subscriber('/control/traffic_light_finished', UInt8, self.cbTrafficLightFinished, queue_size = 1)

        self.pub_traffic_light_return = rospy.Publisher('/detect/traffic_light_stamped', UInt8, queue_size=1)

        self.pub_parking_start = rospy.Publisher('/control/traffic_light_start', UInt8, queue_size = 1)

        # self.pub_traffic_light = rospy.Publisher('/detect/traffic_light', UInt8, queue_size=1)

        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)

        self.StepOfTrafficLight = Enum('StepOfTrafficLight', 'searching_traffic_light searching_green_light searching_yellow_light searching_red_light waiting_green_light pass_traffic_light')

        self.cvBridge = CvBridge()
        self.cv_image = None

        self.stop_bar_count = 0

        self.is_traffic_light_finished = False


        self.mode = 'running'   # tuning, running
        self.detecting_color = 'red'  # red, orange, green

        self.green_count = 0
        self.yellow_count = 0
        self.red_count = 0
        self.stop_count = 0

        self.green_max = 0
        self.yellow_max = 0
        self.red_max = 0

        self.traffic_light_end = "no"
        self.state = "run"


        # rospy.sleep(1)

        # loop_rate = rospy.Rate(15)
        # while not rospy.is_shutdown():
        #     self.fnFindTrafficLight()

        #     loop_rate.sleep()

    def cbGetImage(self, image_msg):
        if self.sub_image_original_type == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        self.fnFindTrafficLight()

    def fnFindTrafficLight(self):
        cv_image_mask = self.fnMaskGreenTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

        ret = self.fnFindCircleOfTrafficLight(cv_image_mask, 'green')

        if ret == 1 or ret == 5:
            self.green_count += 1
        else:
            self.green_count = 0

        cv_image_mask = self.fnMaskYellowTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

        ret = self.fnFindCircleOfTrafficLight(cv_image_mask, 'yellow')
        if ret == 2:
            self.yellow_count += 1
        else:
            self.yellow_count = 0

        cv_image_mask = self.fnMaskRedTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

        ret = self.fnFindCircleOfTrafficLight(cv_image_mask, 'red')
        if ret == 3:
            self.red_count += 1
        elif ret == 4:
            self.red_count = 0
            self.stop_count += 1
        else:
            self.red_count = 0
            self.stop_count = 0


        rospy.loginfo("G:%d Y:%d R:%d", self.green_count, self.yellow_count, self.red_count)

        # if self.green_count > self.green_max:
        #     self.green_max = self.green_count

        # if self.yellow_count > self.yellow_max:
        #     self.yellow_max = self.yellow_count

        # if self.red_count > self.red_max:
        #     self.red_max = self.red_count

        # rospy.loginfo("MAX G:%d Y:%d R:%d", self.green_max, self.yellow_max, self.red_max)

        if self.green_count > 25:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.19
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("GO")

        if self.yellow_count > 25:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.06
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("WARN")

        if self.red_count > 25:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.04
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("STEADY")

        if self.stop_count > 25:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.0
            self.pub_max_vel.publish(msg_pub_max_vel)  
            rospy.loginfo("STOP")


        # msg_pub_max_vel = Float64()
        # msg_pub_max_vel.data = 0.02
        # self.pub_max_vel.publish(msg_pub_max_vel)

        # rospy.loginfo("RED")
        # return 'red'


    def fnMaskRedTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l_red = 132
        Hue_h_red = 180
        Saturation_l_red = 76#138
        Saturation_h_red = 255
        Lightness_l_red = 38#0
        Lightness_h_red = 255

        # if self.showing_trackbar == "on":
        #     cv2.namedWindow('mask_red')
        #     cv2.createTrackbar('Hue_l_red', 'mask_red', Hue_l_red, 179, callback)
        #     cv2.createTrackbar('Hue_h_red', 'mask_red', Hue_h_red, 180, callback)
        #     cv2.createTrackbar('Saturation_l_red', 'mask_red', Saturation_l_red, 254, callback)
        #     cv2.createTrackbar('Saturation_h_red', 'mask_red', Saturation_h_red, 255, callback)
        #     cv2.createTrackbar('Lightness_l_red', 'mask_red', Lightness_l_red, 254, callback)
        #     cv2.createTrackbar('Lightness_h_red', 'mask_red', Lightness_h_red, 255, callback)

        #     # getting homography variables from trackbar
        #     Hue_l_red = cv2.getTrackbarPos('Hue_l_red', 'mask_red')
        #     Hue_h_red = cv2.getTrackbarPos('Hue_h_red', 'mask_red')
        #     Saturation_l_red = cv2.getTrackbarPos('Saturation_l_red', 'mask_red')
        #     Saturation_h_red = cv2.getTrackbarPos('Saturation_h_red', 'mask_red')
        #     Lightness_l_red = cv2.getTrackbarPos('Lightness_l_red', 'mask_red')
        #     Lightness_h_red = cv2.getTrackbarPos('Lightness_h_red', 'mask_red')

        # define range of red color in HSV
        lower_red = np.array([Hue_l_red, Saturation_l_red, Lightness_l_red])
        upper_red = np.array([Hue_h_red, Saturation_h_red, Lightness_h_red])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        # cv2.imshow('frame_red',image), cv2.waitKey(1)
        if self.showing_images == "on":
            cv2.imshow('mask_red',mask), cv2.waitKey(1)
            # cv2.resizeWindow('mask_red', 640, 480)
            # cv2.imshow('res_red',res), cv2.waitKey(1)

        mask = cv2.bitwise_not(mask)

        return mask

    def fnMaskYellowTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l_yellow = 6
        Hue_h_yellow = 31
        Saturation_l_yellow = 13
        Saturation_h_yellow = 255
        Lightness_l_yellow = 70
        Lightness_h_yellow = 255

        if self.showing_trackbar == "on":
            cv2.namedWindow('mask_yellow')
            cv2.createTrackbar('Hue_l_yellow', 'mask_yellow', Hue_l_yellow, 179, callback)
            cv2.createTrackbar('Hue_h_yellow', 'mask_yellow', Hue_h_yellow, 180, callback)
            cv2.createTrackbar('Saturation_l_yellow', 'mask_yellow', Saturation_l_yellow, 254, callback)
            cv2.createTrackbar('Saturation_h_yellow', 'mask_yellow', Saturation_h_yellow, 255, callback)
            cv2.createTrackbar('Lightness_l_yellow', 'mask_yellow', Lightness_l_yellow, 254, callback)
            cv2.createTrackbar('Lightness_h_yellow', 'mask_yellow', Lightness_h_yellow, 255, callback)

            # getting homography variables from trackbar
            Hue_l_yellow = cv2.getTrackbarPos('Hue_l_yellow', 'mask_yellow')
            Hue_h_yellow = cv2.getTrackbarPos('Hue_h_yellow', 'mask_yellow')
            Saturation_l_yellow = cv2.getTrackbarPos('Saturation_l_yellow', 'mask_yellow')
            Saturation_h_yellow = cv2.getTrackbarPos('Saturation_h_yellow', 'mask_yellow')
            Lightness_l_yellow = cv2.getTrackbarPos('Lightness_l_yellow', 'mask_yellow')
            Lightness_h_yellow = cv2.getTrackbarPos('Lightness_h_yellow', 'mask_yellow')

        # define range of yellow color in HSV
        lower_yellow = np.array([Hue_l_yellow, Saturation_l_yellow, Lightness_l_yellow])
        upper_yellow = np.array([Hue_h_yellow, Saturation_h_yellow, Lightness_h_yellow])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        # cv2.imshow('frame_yellow',image), cv2.waitKey(1)
        if self.showing_images == "on":
            cv2.imshow('mask_yellow',mask), cv2.waitKey(1)
            # cv2.resizeWindow('mask_yellow', 640, 480)
            # cv2.imshow('res_yellow',res), cv2.waitKey(1)

        mask = cv2.bitwise_not(mask)

        return mask

    def fnMaskGreenTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l_green = 63
        Hue_h_green = 120
        Saturation_l_green = 116
        Saturation_h_green = 255
        Lightness_l_green = 113
        Lightness_h_green = 255

        # if self.showing_trackbar == "on":
        #     cv2.namedWindow('mask_green')
        #     cv2.createTrackbar('Hue_l_green', 'mask_green', Hue_l_green, 179, callback)
        #     cv2.createTrackbar('Hue_h_green', 'mask_green', Hue_h_green, 180, callback)
        #     cv2.createTrackbar('Saturation_l_green', 'mask_green', Saturation_l_green, 254, callback)
        #     cv2.createTrackbar('Saturation_h_green', 'mask_green', Saturation_h_green, 255, callback)
        #     cv2.createTrackbar('Lightness_l_green', 'mask_green', Lightness_l_green, 254, callback)
        #     cv2.createTrackbar('Lightness_h_green', 'mask_green', Lightness_h_green, 255, callback)

        #     # getting homography variables from trackbar
        #     Hue_l_green = cv2.getTrackbarPos('Hue_l_green', 'mask_green')
        #     Hue_h_green = cv2.getTrackbarPos('Hue_h_green', 'mask_green')
        #     Saturation_l_green = cv2.getTrackbarPos('Saturation_l_green', 'mask_green')
        #     Saturation_h_green = cv2.getTrackbarPos('Saturation_h_green', 'mask_green')
        #     Lightness_l_green = cv2.getTrackbarPos('Lightness_l_green', 'mask_green')
        #     Lightness_h_green = cv2.getTrackbarPos('Lightness_h_green', 'mask_green')

        # define range of green color in HSV
        lower_green = np.array([Hue_l_green, Saturation_l_green, Lightness_l_green])
        upper_green = np.array([Hue_h_green, Saturation_h_green, Lightness_h_green])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        # cv2.imshow('frame_green',image), cv2.waitKey(1)
        if self.showing_images == "on":
            cv2.imshow('mask_green',mask), cv2.waitKey(1)
            # cv2.resizeWindow('mask_green', 640, 480)
            # cv2.imshow('res_green',res), cv2.waitKey(1)

        mask = cv2.bitwise_not(mask)

        return mask

    def fnFindCircleOfTrafficLight(self, mask, find_color):
        status = 0

        params=cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 600

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.6

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.6

        det=cv2.SimpleBlobDetector_create(params)
        keypts=det.detect(mask)
        frame=cv2.drawKeypoints(self.cv_image,keypts,np.array([]),(0,255,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        if self.showing_final_image == "on":
            cv2.imshow('frame', frame), cv2.waitKey(1)

        col1 = 180
        col2 = 270

        col3 = 305

        low1 = 50
        low2 = 170

        low3 = 170
        
        if self.showing_images == "on":
            frame = cv2.line(frame, (col1, low1), (col1, low2), (255, 255, 0), 5)
            frame = cv2.line(frame, (col1, low2), (col2, low2), (255, 255, 0), 5)
            frame = cv2.line(frame, (col2, low2), (col2, low1), (255, 255, 0), 5)
            frame = cv2.line(frame, (col2, low1), (col1, low1), (255, 255, 0), 5)

        # if detected more than 1 red light
        for i in range(len(keypts)):
            point_col = int(keypts[i].pt[0])
            point_low = int(keypts[i].pt[1])

            if point_col > col1 and point_col < col2 and point_low > low1 and point_low < low2:
                if find_color == 'green':
                    status = 1
                elif find_color == 'yellow':
                    status = 2
                elif find_color == 'red':
                    status = 3
            elif point_col > col2 and point_col < col3 and point_low > low1 and point_low < low3:
                if find_color == 'red':
                    status = 4
                elif find_color == 'green':
                    status = 5
            else:
                status = 6

        return status

        # if self.red_count > 0:
        #     self.red_count -= 1

        # if self.detecting_color == 'red':
        #     self.detecting_color = 'green'
        # elif self.detecting_color == 'green':
        #     self.detecting_color = 'red'



        # return is_level_detected, is_level_close, is_level_opened

    def cbTrafficLightFinished(self, traffic_light_finished_msg):
        self.is_traffic_light_finished = True

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_traffic_light')
    node = DetectTrafficLight()
    node.main()
