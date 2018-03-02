#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
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

# Author: Ryu Woon Jung (Leon), [AuTURBO] Ki Hoon Kim (https://github.com/auturbo)

import rospy
import numpy as np
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class ImageProjection():
    def __init__(self):
        self.trackbar = "off"              # "on" / "off"
        self.image_output = "off"          # "on" / "off"
        self.sub_image_type = "raw"        # "compressed" / "raw"
        self.pub_image_type = "raw"        # "compressed" / "raw"

        if self.sub_image_type == "compressed":
            # subscribes compressed image 
            self.sub_image_original = rospy.Subscriber('/camera/image_input/compressed', CompressedImage, self.cbImageProjection, queue_size=1)
        elif self.sub_image_type == "raw":
            # subscribes raw image 
            self.sub_image_original = rospy.Subscriber('/camera/image_input', Image, self.cbImageProjection, queue_size=1)

        if self.pub_image_type == "compressed":
            # publishes ground-project image in compressed type 
            self.pub_image_projected = rospy.Publisher('/camera/image_output/compressed', CompressedImage, queue_size=1)
        elif self.pub_image_type == "raw":
            # publishes ground-project image in raw type 
            self.pub_image_projected = rospy.Publisher('/camera/image_output', Image, queue_size=1)

        self.cvBridge = CvBridge()

    def cbImageProjection(self, msg_img):
        if self.sub_image_type == "compressed":
            # converts compressed image to opencv image
            np_image_original = np.fromstring(msg_img.data, np.uint8)
            cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            # converts raw image to opencv image
            cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        cv_image_balanced = self.simplest_cb(cv_image_original, 20.0)

        if self.image_output == "on":
            # shows original image
            cv2.namedWindow('cv_image_original', cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow('cv_image_original', 0, 250)
            cv2.imshow('cv_image_original', cv_image_original), cv2.waitKey(1)
            
            # shows homography view
            cv2.namedWindow('cv_image_balanced', cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow('cv_image_balanced', 500, 250)
            cv2.imshow('cv_image_balanced', cv_image_balanced), cv2.waitKey(1)

        if self.pub_image_type == "compressed":
            # publishes ground-project image in compressed type
            self.pub_image_projected.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_balanced, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes ground-project image in raw type
            self.pub_image_projected.publish(self.cvBridge.cv2_to_imgmsg(cv_image_balanced, "bgr8"))



    def apply_mask(self, matrix, mask, fill_value):
        masked = np.ma.array(matrix, mask=mask, fill_value=fill_value)
        return masked.filled()

    def apply_threshold(self, matrix, low_value, high_value):
        low_mask = matrix < low_value
        matrix = self.apply_mask(matrix, low_mask, low_value)

        high_mask = matrix > high_value
        matrix = self.apply_mask(matrix, high_mask, high_value)

        return matrix

    def simplest_cb(self, img, percent):
        assert img.shape[2] == 3
        assert percent > 0 and percent < 100

        half_percent = percent / 200.0

        channels = cv2.split(img)

        out_channels = []
        for channel in channels:
            assert len(channel.shape) == 2
            # find the low and high precentile values (based on the input percentile)
            height, width = channel.shape
            vec_size = width * height
            flat = channel.reshape(vec_size)

            assert len(flat.shape) == 1

            flat = np.sort(flat)

            n_cols = flat.shape[0]

            low_val  = flat[math.floor(n_cols * half_percent)]
            high_val = flat[math.ceil( n_cols * (1.0 - half_percent))]

            print "Lowval: ", low_val
            print "Highval: ", high_val

            # saturate below the low percentile and above the high percentile
            thresholded = self.apply_threshold(channel, low_val, high_val)
            # scale the channel
            normalized = cv2.normalize(thresholded, thresholded.copy(), 0, 255, cv2.NORM_MINMAX)
            out_channels.append(normalized)

        return cv2.merge(out_channels)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_projection')
    node = ImageProjection()
    node.main()
