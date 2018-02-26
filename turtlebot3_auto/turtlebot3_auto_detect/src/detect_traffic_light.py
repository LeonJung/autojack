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
        self.showing_final_image = "on"
        self.showing_trackbar = "off"
        self.sub_image_original_type = "raw" # you can choose image type "compressed", "raw"
        self.showing_images = "off" # you can choose showing images or not by "on", "off"

        if self.sub_image_original_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.cbGetImage, queue_size = 1)
        elif self.sub_image_original_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.cbGetImage, queue_size = 1)
 
        self.sub_traffic_light_order = rospy.Subscriber('/detect/traffic_light_order', UInt8, self.cbTrafficLightOrder, queue_size=1)

        self.sub_traffic_light_finished = rospy.Subscriber('/control/traffic_light_finished', UInt8, self.cbTrafficLightFinished, queue_size = 1)

        self.pub_traffic_light_return = rospy.Publisher('/detect/traffic_light_stamped', UInt8, queue_size=1)

        self.pub_parking_start = rospy.Publisher('/control/traffic_light_start', UInt8, queue_size = 1)

        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)

        self.StepOfTrafficLight = Enum('StepOfTrafficLight', 'searching_traffic_light searching_green_light searching_yellow_light searching_red_light waiting_green_light pass_traffic_light')

        self.Hue_l_red = 0
        self.Hue_h_red = 8
        self.Saturation_l_red = 138
        self.Saturation_h_red = 255
        self.Lightness_l_red = 0
        self.Lightness_h_red = 255

        self.cvBridge = CvBridge()
        self.cv_image = None

        self.stop_bar_count = 0

        self.is_traffic_light_finished = False

        rospy.sleep(1)

        loop_rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.fnFindTrafficLight()

            loop_rate.sleep()

    def cbGetImage(self, image_msg):
        if self.sub_image_original_type == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

    def cbTrafficLightOrder(self, order):
        pub_traffic_light_return = UInt8()
    
        if order.data == self.StepOfTrafficLight.searching_traffic_light.value:
            rospy.loginfo("Now lane_following")

            pub_traffic_light_return.data = self.StepOfTrafficLight.searching_traffic_light.value
                            
                                
        elif order.data == self.StepOfTrafficLight.searching_green_light.value:
            rospy.loginfo("Now searching_green_light")

            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.10
            self.pub_max_vel.publish(msg_pub_max_vel)

            while True:
                is_green_light_detected, _, _ = self.fnFindTrafficLight()
                if is_green_light_detected == True:
                    break
                else:
                    pass
            
            rospy.loginfo("SLOWDOWN!!")

            msg_pub_max_vel.data = 0.04
            self.pub_max_vel.publish(msg_pub_max_vel)

            pub_traffic_light_return.data = self.StepOfTrafficLight.searching_green_light.value


        elif order.data == self.StepOfTrafficLight.searching_yellow_light.value:
            rospy.loginfo("Now searching_yellow_light")

            while True:
                _, is_yellow_light_detected, _ = self.fnFindTrafficLight()
                if is_yellow_light_detected == True:
                    break
                else:
                    pass
            rospy.loginfo("STOP~~")

            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.0
            self.pub_max_vel.publish(msg_pub_max_vel)

            pub_traffic_light_return.data = self.StepOfTrafficLight.searching_yellow_light.value


        elif order.data == self.StepOfTrafficLight.searching_red_light.value:
            rospy.loginfo("Now searching_red_light")

            while True:
                _, _, is_red_light_detected = self.fnFindTrafficLight()
                if is_red_light_detected == True:
                    break
                else:
                    pass

            rospy.loginfo("GO~~")

            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.19
            self.pub_max_vel.publish(msg_pub_max_vel)

            pub_traffic_light_return.data = self.StepOfTrafficLight.searching_red_light.value

        elif order.data == self.StepOfTrafficLight.waiting_green_light.value:
            rospy.loginfo("Now waiting_green_light")

            while True:
                is_green_light_detected, _, _ = self.fnFindTrafficLight()
                if is_green_light_detected == True:
                    break
                else:
                    pass

            rospy.loginfo("GO~~")

            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.19
            self.pub_max_vel.publish(msg_pub_max_vel)

            pub_traffic_light_return.data = self.StepOfTrafficLight.waiting_green_light.value

        elif order.data == self.StepOfTrafficLight.pass_traffic_light.value:
            rospy.loginfo("Now pass_traffic_light")

            # while True:
            #     if self.is_obstacle_detected == False:
            #         rospy.loginfo("parking lot is clear")
            #         break
            #     else:
            #         rospy.loginfo("right side is full")

            pub_traffic_light_return.data = self.StepOfTrafficLight.pass_traffic_light.value

        self.pub_traffic_light_return.publish(pub_traffic_light_return)

    def fnFindTrafficLight(self):

        cv_image_mask = self.fnMaskRedOfLevel()

        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

        return self.fnFindRectOfLevel(cv_image_mask)

    def fnMaskRedOfLevel(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.Hue_l_red
        Hue_h = self.Hue_h_red
        Saturation_l = self.Saturation_l_red
        Saturation_h = self.Saturation_h_red
        Lightness_l = self.Lightness_l_red
        Lightness_h = self.Lightness_h_red

        if self.showing_trackbar == "on":
            cv2.namedWindow('mask_red')
            cv2.createTrackbar('Hue_l', 'mask_red', Hue_l, 179, callback)
            cv2.createTrackbar('Hue_h', 'mask_red', Hue_h, 180, callback)
            cv2.createTrackbar('Saturation_l', 'mask_red', Saturation_l, 254, callback)
            cv2.createTrackbar('Saturation_h', 'mask_red', Saturation_h, 255, callback)
            cv2.createTrackbar('Lightness_l', 'mask_red', Lightness_l, 254, callback)
            cv2.createTrackbar('Lightness_h', 'mask_red', Lightness_h, 255, callback)

            # getting homography variables from trackbar
            Hue_l = cv2.getTrackbarPos('Hue_l', 'mask_red')
            Hue_h = cv2.getTrackbarPos('Hue_h', 'mask_red')
            Saturation_l = cv2.getTrackbarPos('Saturation_l', 'mask_red')
            Saturation_h = cv2.getTrackbarPos('Saturation_h', 'mask_red')
            Lightness_l = cv2.getTrackbarPos('Lightness_l', 'mask_red')
            Lightness_h = cv2.getTrackbarPos('Lightness_h', 'mask_red')

        # define range of red color in HSV
        lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_red = np.array([Hue_h, Saturation_h, Lightness_h])


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


    def fnFindRectOfLevel(self, mask):
        is_level_detected = False
        is_level_close = False
        is_level_opened = False

        params=cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 3000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.5

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.9

        det=cv2.SimpleBlobDetector_create(params)
        keypts=det.detect(mask)
        frame=cv2.drawKeypoints(self.cv_image,keypts,np.array([]),(0,255,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        mean_x = 0.0
        mean_y = 0.0

        # if detected 3 red rectangular
        if len(keypts) == 3:
            for i in range(3):
                mean_x = mean_x + keypts[i].pt[0]/3
                mean_y = mean_y + keypts[i].pt[1]/3
            arr_distances = [0]*3
            for i in range(3):
                arr_distances[i] = fnCalcDistanceDot2Dot(mean_x, mean_y, keypts[i].pt[0], keypts[i].pt[1])

            # finding thr farthest point from the center
            idx1, idx2, idx3 = fnArrangeIndexOfPoint(arr_distances)
            frame = cv2.line(frame, (int(keypts[idx1].pt[0]), int(keypts[idx1].pt[1])), (int(keypts[idx2].pt[0]), int(keypts[idx2].pt[1])), (255, 0, 0), 5)
            frame = cv2.line(frame, (int(mean_x), int(mean_y)), (int(mean_x), int(mean_y)), (255, 255, 0), 5)
            point1 =  [int(keypts[idx1].pt[0]), int(keypts[idx1].pt[1]-1)]
            point2 = [int(keypts[idx3].pt[0]), int(keypts[idx3].pt[1]-1)]
            point3 = [int(keypts[idx2].pt[0]), int(keypts[idx2].pt[1]-1)]

            # test linearity and distance equality. If satisfy the both, continue to next process
            is_rects_linear = fnCheckLinearity(point1, point2, point3)
            is_rects_dist_equal = fnCheckDistanceIsEqual(point1, point2, point3)

            if is_rects_linear == True or is_rects_dist_equal == True:
                # finding the angle of line
                # angle = math.atan2(keypts[idx1].pt[1] - keypts[idx2].pt[1], keypts[idx1].pt[0] - keypts[idx2].pt[0]) * 180 / 3.141592
                distance_bar2car = 100 / fnCalcDistanceDot2Dot(point1[0], point1[1], point2[0], point2[1])
                # if angle < 0:
                #     angle = angle + 180
                # if angle > 90:
                #     angle = 180 - angle

                # # publishing topic
                # if angle < 45:
                self.stop_bar_count = 40
                if distance_bar2car > 1.5:
                    is_level_detected = True
                    self.stop_bar_state = 'slowdown'
                    self.state = "detected"
                else:
                    is_level_close = True
                    self.stop_bar_state = 'stop'

                # print distance_bar2car
                # else:
                #     rospy.loginfo("GO~~")
                #     self.stop_bar_count = 0
                #     self.stop_bar_state = 'go'

                # cv2.putText(frame, self.stop_bar_state,(0, 0), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2)
        #         message = Stop_bar()
        #         message.state = self.stop_bar_state
        #         message.distance = 100 / find_distance_dot2dot(point1[0], point1[1], point2[0], point2[1])
        #         message.position1_x = point1[0]
        #         message.position1_y = point1[1]
        #         message.position2_x = point3[0]
        #         message.position2_y = point3[1]
        #         self._pub.publish(message)

        if self.stop_bar_count > 0:
            self.stop_bar_count -= 1
        if self.stop_bar_count == 1:
            is_level_opened = True
            self.stop_bar_state = 'go'
        #     message = Stop_bar()
        #     message.state = self.stop_bar_state
        #     message.distance = 0
        #     message.position1_x = 0
        #     message.position1_y = 0
        #     message.position2_x = 0
        #     message.position2_y = 0
        #     self._pub.publish(message)

        if self.showing_final_image == "on":
            cv2.imshow('frame', frame), cv2.waitKey(1)

        return is_level_detected, is_level_close, is_level_opened

    def cbTrafficLightFinished(self, traffic_light_finished_msg):
        self.is_traffic_light_finished = True

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_traffic_light')
    node = DetectTrafficLight()
    node.main()
