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

# Author: Ryu Woon Jung (Leon)

import rospy
import cv2
import numpy as np
from sklearn.cluster import KMeans
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image, CompressedImage
import tf

import matplotlib.pyplot as plt

import math
from enum import Enum

def callback(x):
    pass

class DetectSign():
    def __init__(self):
        self.fnPreproc()

        self.showing_plot_track = "off"
        self.showing_images = "off" # you can choose showing images or not by "on", "off"

        self.sub_image_original_type = "raw" # you can choose image type "compressed", "raw"
        self.pub_image_traffic_sign_type = "raw" # you can choose image type "compressed", "raw"

        if self.sub_image_original_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.callback, queue_size = 1)
        elif self.sub_image_original_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.callback, queue_size = 1)

        self.pub_sign = rospy.Publisher('/detect/traffic_sign', UInt8, queue_size=1)

        if self.pub_image_traffic_sign_type == "compressed":
            self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output/compressed', CompressedImage, queue_size = 1)
        elif self.pub_image_traffic_sign_type == "raw":
            self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output', Image, queue_size = 1)

        self.pub_traffic_sign = rospy.Publisher('/detect/traffic_sign', UInt8, queue_size = 1)

        self.cvBridge = CvBridge()

        self.TrafficSign = Enum('TrafficSign', 'divide stop parking tunnel')

        self.counter = 0

        self.RECOG_MIN_COUNT = 3

        self.recog_counter_1 = 0
        self.recog_counter_2 = 0
        self.recog_counter_3 = 0
        self.recog_counter_4 = 0


    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create()

        self.img2 = cv2.imread('/home/leon/Desktop/divide.png',0) # trainImage1
        self.img3 = cv2.imread('/home/leon/Desktop/stop.png',0) # trainImage2
        self.img4 = cv2.imread('/home/leon/Desktop/parking.png',0) # trainImage3
        self.img5 = cv2.imread('/home/leon/Desktop/tunnel.png',0) # trainImage4

        self.kp2, self.des2 = self.sift.detectAndCompute(self.img2,None)
        self.kp3, self.des3 = self.sift.detectAndCompute(self.img3,None)
        self.kp4, self.des4 = self.sift.detectAndCompute(self.img4,None)
        self.kp5, self.des5 = self.sift.detectAndCompute(self.img5,None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #img1 and 2 should have same shape
            err = sum / num_all
            return err


    def callback(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        # if self.counter % 3 != 0:
        #     self.counter += 1
        #     return

        if self.sub_image_original_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            img1 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_original_type == "raw":
            img1 = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        MIN_MATCH_COUNT = 9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(img1,None)

        matches1 = self.flann.knnMatch(des1,self.des2,k=2)
        matches2 = self.flann.knnMatch(des1,self.des3,k=2)
        matches3 = self.flann.knnMatch(des1,self.des4,k=2)
        matches4 = self.flann.knnMatch(des1,self.des5,k=2)

        # store all the good matches as per Lowe's ratio test.
        good1 = []
        for m,n in matches1:
            if m.distance < 0.7*n.distance:
                good1.append(m)

        if len(good1)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good1 ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp2[m.trainIdx].pt for m in good1 ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask1 = mask.ravel().tolist()

            # h = img1.shape[0]
            # w = img1.shape[1]
            # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # dst = cv2.perspectiveTransform(pts,M)

            # self.img2 = cv2.polylines(self.img2,[np.int32(dst)],True,(255, 0, 0))#,3, cv2.LINE_AA)
            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                self.recog_counter_1 += 1

                # rospy.loginfo("Found1! %d %d", self.recog_counter_1, mse)

                if self.recog_counter_1 == self.RECOG_MIN_COUNT:
                    self.recog_counter_1 = 0
                    msg_sign = UInt8()
                    msg_sign.data = self.TrafficSign.divide.value

                    self.pub_sign.publish(msg_sign)

                    # rospy.loginfo("TrafficSign 1")


        else:
            # print "Not enough matches are found 1 - %d/%d" % (len(good1),MIN_MATCH_COUNT)
            self.recog_counter_1 = 0
            matchesMask1 = None

        good2 = []
        for m,n in matches2:
            if m.distance < 0.7*n.distance:
                good2.append(m)

        if len(good2)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good2 ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp3[m.trainIdx].pt for m in good2 ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask2 = mask.ravel().tolist()

            # h = img1.shape[0]
            # w = img1.shape[1]
            # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # dst = cv2.perspectiveTransform(pts,M)

            # self.img3 = cv2.polylines(self.img3,[np.int32(dst)],True,255,3, cv2.LINE_AA)
            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                self.recog_counter_2 += 1

                # rospy.loginfo("Found2! %d %d", self.recog_counter_2, mse)

                if self.recog_counter_2 == self.RECOG_MIN_COUNT:
                    self.recog_counter_2 = 0
                    msg_sign = UInt8()
                    msg_sign.data = self.TrafficSign.stop.value

                    self.pub_sign.publish(msg_sign)

                    # rospy.loginfo("TrafficSign 2")


        else:
            # print "Not enough matches are found 2 - %d/%d" % (len(good2),MIN_MATCH_COUNT)
            self.recog_counter_2 = 0
            matchesMask2 = None

        good3 = []
        for m,n in matches3:
            if m.distance < 0.7*n.distance:
                good3.append(m)

        if len(good3)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good3 ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp4[m.trainIdx].pt for m in good3 ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask3 = mask.ravel().tolist()

            # h = img1.shape[0]
            # w = img1.shape[1]
            # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # dst = cv2.perspectiveTransform(pts,M)

            # self.img3 = cv2.polylines(self.img3,[np.int32(dst)],True,255,3, cv2.LINE_AA)
            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                self.recog_counter_3 += 1

                # rospy.loginfo("Found3! %d %d", self.recog_counter_3, mse)

                if self.recog_counter_3 == self.RECOG_MIN_COUNT:
                    self.recog_counter_3 = 0
                    msg_sign = UInt8()
                    msg_sign.data = self.TrafficSign.parking.value

                    self.pub_sign.publish(msg_sign)

                    # rospy.loginfo("TrafficSign 3")


        else:
            # print "Not enough matches are found 3 - %d/%d" % (len(good3),MIN_MATCH_COUNT)
            self.recog_counter_3 = 0
            matchesMask3 = None

        good4 = []
        for m,n in matches4:
            if m.distance < 0.7*n.distance:
                good4.append(m)

        if len(good4)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good4 ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp5[m.trainIdx].pt for m in good4 ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask4 = mask.ravel().tolist()

            # h = img1.shape[0]
            # w = img1.shape[1]
            # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # dst = cv2.perspectiveTransform(pts,M)

            # self.img3 = cv2.polylines(self.img3,[np.int32(dst)],True,255,3, cv2.LINE_AA)
            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                self.recog_counter_4 += 1

                # rospy.loginfo("Found4! %d %d", self.recog_counter_4, mse)

                if self.recog_counter_4 == self.RECOG_MIN_COUNT:
                    self.recog_counter_4 = 0
                    msg_sign = UInt8()
                    msg_sign.data = self.TrafficSign.tunnel.value

                    self.pub_sign.publish(msg_sign)

                    # rospy.loginfo("TrafficSign 4")

        else:
            # print "Not enough matches are found 4 - %d/%d" % (len(good4),MIN_MATCH_COUNT)
            self.recog_counter_4 = 0
            matchesMask4 = None


        draw_params1 = dict(matchColor = (0,255,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matchesMask1, # draw only inliers
                        flags = 2)

        final1 = cv2.drawMatches(img1,kp1,self.img2,self.kp2,good1,None,**draw_params1)
        
        draw_params2 = dict(matchColor = (0,0,255), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matchesMask2, # draw only inliers
                        flags = 2)

        final2 = cv2.drawMatches(img1,kp1,self.img3,self.kp3,good2,None,**draw_params2)

        draw_params3 = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matchesMask3, # draw only inliers
                        flags = 2)

        final3 = cv2.drawMatches(img1,kp1,self.img4,self.kp4,good3,None,**draw_params3)

        draw_params4 = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matchesMask4, # draw only inliers
                        flags = 2)

        final4 = cv2.drawMatches(img1,kp1,self.img5,self.kp5,good4,None,**draw_params4)


        if self.showing_images == "on":
            # cv2.imshow('img1', img1), cv2.waitKey(1)
            cv2.imshow('final1', final1), cv2.waitKey(1)
            cv2.imshow('final2', final2), cv2.waitKey(1)
            cv2.imshow('final3', final3), cv2.waitKey(1)
            cv2.imshow('final4', final4), cv2.waitKey(1)

        # publishing calbrated and Bird's eye view as compressed image
        # if self.pub_image_lane_type == "compressed":
            # msg_final_img = CompressedImage()
            # msg_final_img.header.stamp = rospy.Time.now()
            # msg_final_img.format = "jpeg"
            # msg_final_img.data = np.array(cv2.imencode('.jpg', final)[1]).tostring()
            # self.pub_image_lane.publish(msg_final_img)

            # self._pub4.publish(msg_off_center)

            # msg_homography = CompressedImage()
            # msg_homography.header.stamp = rospy.Time.now()
            # msg_homography.format = "jpeg"
            # msg_homography.data = np.array(cv2.imencode('.jpg', cv_Homography)[1]).tostring()
            # self.pub_image_lane.publish(msg_homography)

        # publishing calbrated and Bird's eye view as raw image
        # elif self.pub_image_lane_type == "raw":
            # self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, "bgr8"))

            # print(msg_desired_center.data)

            # msg_off_center = Float64()
            # msg_off_center.data = off_center

            # self._pub4.publish(msg_off_center)

            # self._pub4.publish(self.bridge.cv2_to_imgmsg(cv_Homography, "bgr8"))
            # self._pub4.publish(self.bridge.cv2_to_imgmsg(cv_Homography, "mono8"))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_sign')
    node = DetectSign()
    node.main()



















