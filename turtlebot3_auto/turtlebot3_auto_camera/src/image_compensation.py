#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage

# from dynamic_reconfigure.server import Server as DynamicReconfigureServer
# from turtlebot3_auto_camera.cfg import ImageCompensationParamsConfig

class ImageCompensation():
    def __init__(self):
        # drs = DynamicReconfigureServer(ImageCompensationParamsConfig, self.cbDRImageCompensationParams)

        self.show_image_cv2window = "off"               # "on" / "off"
        self.sub_image_original_type = "compressed"     # "compressed" / "raw"
        self.pub_image_compensated_type = "compressed"  # "compressed" / "raw"

        if self.sub_image_original_type == "compressed":
            # subscribes compressed image 
            self.sub_image_original = rospy.Subscriber('/camera/image_input/compressed', CompressedImage, self.cbImageCompensation, queue_size = 1)
        elif self.sub_image_original_type == "raw":
            # subscribes raw image 
            self.sub_image_original = rospy.Subscriber('/camera/image_input', Image, self.cbImageCompensation, queue_size = 1)

        if self.pub_image_compensated_type == "compressed":
            # publishes compensated image in compressed type 
            self.pub_image_compensated = rospy.Publisher('/camera/image_output/compressed', CompressedImage, queue_size = 1)
        elif self.pub_image_compensated_type == "raw":
            # publishes compensated image in raw type
            self.pub_image_compensated = rospy.Publisher('/camera/image_output', Image, queue_size = 1)

        self.cvBridge = CvBridge()

        self.counter = 0
        
    def cbImageCompensation(self, msg_img):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        # if self.counter % 3 != 0:
        #     self.counter += 1
        #     return

        if self.sub_image_original_type == "compressed":
            # converts compressed image to opencv image
            np_image_original = np.fromstring(msg_img.data, np.uint8)
            cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        elif self.sub_image_original_type == "raw":
            # converts raw image to opencv image
            cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        cv_image_compensated = np.copy(cv_image_original)

        ## Image compensation based on not-originalinal histogram equalization method
        # TODO : processing speed
        # TODO : should this be placed after image_to_ground image? or rectified image? if it calculates based on rectified image, the brightness varies on so much stuffs around
        self.clip_hist_percent = 1.0
        
        hist_size = 256
        alpha = 0
        beta = 0
        min_gray = 0
        max_gray = 0

        gray = cv2.cvtColor(cv_image_compensated, cv2.COLOR_BGR2GRAY)

        if self.clip_hist_percent == 0.0:
            min_gray, max_gray, _, _ = cv2.minMaxLoc(gray)
        else:
            hist = cv2.calcHist([gray], [0], None, [hist_size], [0, hist_size])

            accumulator = np.zeros((hist_size))
            
            accumulator[0] = hist[0]

            for i in range(0, hist_size):
                accumulator[i] = accumulator[i - 1] + hist[i]

            max = accumulator[hist_size - 1]

            self.clip_hist_percent *= (max / 100.)
            self.clip_hist_percent /= 2.

            min_gray = 0
            while accumulator[min_gray] < self.clip_hist_percent:
                min_gray += 1
            
            max_gray = hist_size - 1
            while accumulator[max_gray] >= (max - self.clip_hist_percent):
                max_gray -= 1

        input_range = max_gray - min_gray

        alpha = (hist_size - 1) / input_range
        beta = -min_gray * alpha

        cv_image_compensated = cv2.convertScaleAbs(cv_image_compensated, -1, alpha, beta)

        if self.show_image_cv2window == "on":
            cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow('image', 0, 250)
            cv2.imshow('image', cv_image_original), cv2.waitKey(1)

            cv2.namedWindow('dst', cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow('dst', 800, 500)
            cv2.imshow('dst', cv_image_compensated)

        # publishing calbrated and Bird's eye view as compressed image
        if self.pub_image_compensated_type == "compressed":
            self.pub_image_compensated.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_compensated, "jpg"))

        # publishing calbrated and Bird's eye view as raw image
        elif self.pub_image_compensated_type == "raw":
            self.pub_image_compensated.publish(self.cvBridge.cv2_to_imgmsg(cv_image_compensated, "bgr8"))


    def main(self):
        rospy.spin()

    # def cbDRImageCompensationParams(self, config, level):
    #     self.clip_hist_percent = config["clip_hist_percent"]

    #     rospy.loginfo("%f", self.clip_hist_percent)
    #     # self.a = config["a"]
    #     # self.b = config["b"]

    #     return config

if __name__ == '__main__':
    rospy.init_node('image_compensation')
    node = ImageCompensation()
    node.main()
