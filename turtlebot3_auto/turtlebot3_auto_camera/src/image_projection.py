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
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class ImageProjection():
    def __init__(self):
        self.trackbar = "off"              # "on" / "off"
        self.image_output = "off"          # "on" / "off"
        self.sub_image_type = "compressed"        # "compressed" / "raw"
        self.pub_image_type = "compressed"        # "compressed" / "raw"

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

        # setting homography variables
        Top_w = 56
        Top_h = 50
        Bottom_w = 138
        Bottom_h = 119
        binary_threshold = 150

        if self.trackbar == "on":
            # trackbar
            cv2.namedWindow('cv_image_calib')
            cv2.createTrackbar('Top_w', 'cv_image_calib', Top_w, 120, callback)
            cv2.createTrackbar('Top_h', 'cv_image_calib', Top_h, 120, callback)
            cv2.createTrackbar('Bottom_w', 'cv_image_calib', Bottom_w, 320, callback)
            cv2.createTrackbar('Bottom_h', 'cv_image_calib', Bottom_h, 320, callback)
            cv2.createTrackbar('binary_threshold', 'cv_image_calib', binary_threshold, 255, callback)

            # getting homography variables from trackbar
            Top_w = cv2.getTrackbarPos('Top_w', 'cv_image_calib')
            Top_h = cv2.getTrackbarPos('Top_h', 'cv_image_calib')
            Bottom_w = cv2.getTrackbarPos('Bottom_w', 'cv_image_calib')
            Bottom_h = cv2.getTrackbarPos('Bottom_h', 'cv_image_calib')
            binary_threshold = cv2.getTrackbarPos('binary_threshold', 'cv_image_calib')

        if self.image_output == "on":
            # copy original image to use for cablibration
            cv_image_calib = np.copy(cv_image_original)

            # draw lines to help setting homography variables
            cv_image_calib = cv2.line(cv_image_calib, (160 - Top_w, 180 - Top_h), (160 + Top_w, 180 - Top_h), (0, 0, 255), 1)
            cv_image_calib = cv2.line(cv_image_calib, (160 - Bottom_w, 120 + Bottom_h), (160 + Bottom_w, 120 + Bottom_h), (0, 0, 255), 1)
            cv_image_calib = cv2.line(cv_image_calib, (160 + Bottom_w, 120 + Bottom_h), (160 + Top_w, 180 - Top_h), (0, 0, 255), 1)
            cv_image_calib = cv2.line(cv_image_calib, (160 - Bottom_w, 120 + Bottom_h), (160 - Top_w, 180 - Top_h), (0, 0, 255), 1)

        # adding Gaussian blur to the image of original
        cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        ## homography transform process
        # selecting 4 points from the original image
        pts_src = np.array([[160 - Top_w, 180 - Top_h], [160 + Top_w, 180 - Top_h], [160 + Bottom_w, 120 + Bottom_h], [160 - Bottom_w, 120 + Bottom_h]])

        # selecting 4 points from image that will be transformed
        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

        # finding homography matrix
        h, status = cv2.findHomography(pts_src, pts_dst)

        # homography process
        cv_image_homography = cv2.warpPerspective(cv_image_original, h, (1000, 600))

        # fill the empty space with black triangles on left and right side of bottom
        triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
        triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
        black = (0, 0, 0)
        white = (255, 255, 255)
        cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)

        if self.image_output == "on":
            # shows original image
            cv2.namedWindow('cv_image_original', cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow('cv_image_original', 0, 250)
            cv2.imshow('cv_image_original', cv_image_original), cv2.waitKey(1)
            
            # shows homography view
            cv2.namedWindow('cv_image_homography', cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow('cv_image_homography', 500, 250)
            cv2.imshow('cv_image_homography', cv_image_homography), cv2.waitKey(1)

        if self.pub_image_type == "compressed":
            # publishes ground-project image in compressed type
            self.pub_image_projected.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_homography, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes ground-project image in raw type
            self.pub_image_projected.publish(self.cvBridge.cv2_to_imgmsg(cv_image_homography, "bgr8"))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_projection')
    node = ImageProjection()
    node.main()
