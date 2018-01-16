#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import matplotlib.pyplot as plt

def callback(x):    
    pass

class ImageTransform():
    def __init__(self):
        self.trackbar = "on" # you can choose showing trackbar or not by "on", "off"
        self.showing_images = "on" # you can choose showing images or not by "on", "off"
        self.selecting_sub_image = "compressed" # you can choose image type "compressed", "raw"
        self.selecting_pub_image = "raw" # you can choose image type "compressed", "raw"

        if self.selecting_sub_image == "compressed":
            self._sub = rospy.Subscriber('/camera/image_rect_color/compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub = rospy.Subscriber('/camera/image_rect_color', Image, self.callback, queue_size=1)

        # There are 4 Publishers
        # pub1 : calibrated image as compressed image
        # pub2 : Bird's eye view image as compressed image
        # pub3 : calibrated image as raw image
        # pub4 : Bird's eye view image as raw image
        # if you do not want to publish some topic, delete pub definition in __init__ function and publishing process in callback function
        self._pub1 = rospy.Publisher('/image_calibrated_compressed', CompressedImage, queue_size=1)
        self._pub2 = rospy.Publisher('/image_birdseye_compressed', CompressedImage, queue_size=1)
        self._pub3 = rospy.Publisher('/image_calibrated', Image, queue_size=1)
        self._pub4 = rospy.Publisher('/image_birdseye', Image, queue_size=1)

        self.bridge = CvBridge()

        self.counter = 0

    def callback(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        self.counter += 1
        if self.counter%3 != 0:
            return

        if self.selecting_sub_image == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.selecting_sub_image == "raw":
            cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # #calibration variables
        # fx = 412.939491
        # fy = 411.722476
        # cx = 309.638220
        # cy = 237.289666

        # k1 = -0.431252
        # k2 = 0.138689
        # p1 = -0.009537
        # p2 = 0.001562

        # # camera calibration process
        # camera_matrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
        # distortion_matrix = np.array([k1,k2,p1,p2,0])
        # cv_image = cv2.undistort(cv_image, camera_matrix, distortion_matrix)

        # adding Gaussian blur
        cv_image = cv2.GaussianBlur(cv_image,(5,5),0)

        # copy original image to use in homography process
        cv_origin = np.copy(cv_image)

        # setting homography variables
        Top_w = 56
        Top_h = 50
        Bottom_w = 138
        Bottom_h = 119
        binary_threshold = 150


        if self.trackbar == "on":
            # creating trackbar
            cv2.namedWindow('cv_image')
            cv2.createTrackbar('Top_w', 'cv_image', Top_w, 120, callback)
            cv2.createTrackbar('Top_h', 'cv_image', Top_h, 120, callback)
            cv2.createTrackbar('Bottom_w', 'cv_image', Bottom_w, 320, callback)
            cv2.createTrackbar('Bottom_h', 'cv_image', Bottom_h, 320, callback)
            #             cv2.createTrackbar('Top_w', 'cv_image', Top_w, 240, callback)
            # cv2.createTrackbar('Top_h', 'cv_image', Top_h, 240, callback)
            # cv2.createTrackbar('Bottom_w', 'cv_image', Bottom_w, 320, callback)
            # cv2.createTrackbar('Bottom_h', 'cv_image', Bottom_h, 320, callback)
            cv2.createTrackbar('binary_threshold', 'cv_image', binary_threshold, 255, callback)

            # getting homography variables from trackbar
            Top_w = cv2.getTrackbarPos('Top_w', 'cv_image')
            Top_h = cv2.getTrackbarPos('Top_h', 'cv_image')
            Bottom_w = cv2.getTrackbarPos('Bottom_w', 'cv_image')
            Bottom_h = cv2.getTrackbarPos('Bottom_h', 'cv_image')
            binary_threshold = cv2.getTrackbarPos('binary_threshold', 'cv_image')

        if self.showing_images == "on":
            # draw lines to help setting homography variables
            cv_image = cv2.line(cv_image, (160-Top_w, 180-Top_h), (160+Top_w, 180-Top_h), (0, 0, 255), 1)
            cv_image = cv2.line(cv_image, (160-Bottom_w, 120+Bottom_h), (160+Bottom_w, 120+Bottom_h), (0, 0, 255), 1)
            cv_image = cv2.line(cv_image, (160+Bottom_w, 120+Bottom_h), (160+Top_w, 180-Top_h), (0, 0, 255), 1)
            cv_image = cv2.line(cv_image, (160-Bottom_w, 120+Bottom_h), (160-Top_w, 180-Top_h), (0, 0, 255), 1)

            #             cv_image = cv2.line(cv_image, (320-Top_w, 340-Top_h), (320+Top_w, 340-Top_h), (0, 0, 255), 1)
            # cv_image = cv2.line(cv_image, (320-Bottom_w, 240+Bottom_h), (320+Bottom_w, 240+Bottom_h), (0, 0, 255), 1)
            # cv_image = cv2.line(cv_image, (320+Bottom_w, 240+Bottom_h), (320+Top_w, 340-Top_h), (0, 0, 255), 1)
            # cv_image = cv2.line(cv_image, (320-Bottom_w, 240+Bottom_h), (320-Top_w, 340-Top_h), (0, 0, 255), 1)

        ### homography transform process ###
        # selecting 4 points from the original image
        pts_src = np.array([[160-Top_w, 180-Top_h], [160+Top_w, 180-Top_h], [160+Bottom_w, 120+Bottom_h], [160-Bottom_w, 120+Bottom_h]])
        # selecting 4 points from image that will be transformed
        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])
        # finding homography matrix
        h, status = cv2.findHomography(pts_src, pts_dst)
        # homography process
        cv_Homography = cv2.warpPerspective(cv_origin, h, (1000, 600))
        # fill the empty space with black triangles
        triangle1 = np.array([ [0,599], [0,340], [200,599] ], np.int32)
        triangle2 = np.array([ [999,599], [999,340], [799,599] ], np.int32)
        black = (0,0,0)
        white = (255,255,255)
        cv_Homography = cv2.fillPoly(cv_Homography, [triangle1, triangle2], black)




        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_Homography, cv2.COLOR_BGR2HSV)

        # define range of yellow color in HSV
        # lower_yellow = np.array([30,100,100])
        # upper_yellow = np.array([95,255,255])
        lower_yellow = np.array([30,80,80])
        upper_yellow = np.array([115,255,255])


        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(cv_Homography,cv_Homography, mask = mask)

        # cv2.imshow('frame',cv_Homography)
        # cv2.imshow('mask',mask)
        # cv2.imshow('res',res)



        # #-----Converting image to LAB Color model----------------------------------- 
        # lab= cv2.cvtColor(cv_Homography, cv2.COLOR_BGR2LAB)
        # # cv2.imshow("lab",lab)

        # #-----Splitting the LAB image to different channels-------------------------
        # l, a, b = cv2.split(lab)
        # # cv2.imshow('l_channel', l)
        # # cv2.imshow('a_channel', a)
        # # cv2.imshow('b_channel', b)

        # # #-----Applying CLAHE to L-channel-------------------------------------------
        # clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        # cl = clahe.apply(l)
        # # cv2.imshow('CLAHE output', cl)

        # # #-----Merge the CLAHE enhanced L-channel with the a and b channel-----------
        # limg = cv2.merge((cl,a,b))
        # # cv2.imshow('limg', limg)

        # # #-----Converting image from LAB Color model to RGB model--------------------
        # cv_Homography = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        # # cv2.imshow('final', final)

        # pct = 0.25
        # newsize = (int(cv_image.shape[0] * pct), int(cv_image.shape[1] * pct))
        # small_img = cv2.resize(cv_image, newsize)  

        # image = cv2.cvtColor(small_img, cv2.COLOR_BGR2RGB)

        # homo_1 = image.reshape((image.shape[0] * image.shape[1], 3))
        # clt = KMeans(n_clusters = 4)
        # clt.fit(homo_1)


        # # build a histogram of clusters and then create a figure
        # # representing the number of pixels labeled to each color
        # hist = utils.centroid_histogram(clt)
        # bar = utils.plot_colors(hist, clt.cluster_centers_)
        
        # # show our color bart
        # plt.figure()
        # plt.axis("off")
        # plt.imshow(bar)
        # plt.show()

#### this resizes the picture
        # pct = 0.50
        # newsize = (int(cv_Homography.shape[1] * pct), int(cv_Homography.shape[0] * pct))
        # cv_Homography = cv2.resize(cv_Homography, newsize)  

        # # img = cv2.imread('home.jpg')
        # Z = cv_Homography.reshape((-1,3))
        # # convert to np.float32
        # Z = np.float32(Z)
        # # define criteria, number of clusters(K) and apply kmeans()
        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        # K = 4
        # ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
        # # Now convert back into uint8, and make original cv_image
        # center = np.uint8(center)
        # res = center[label.flatten()]
        # res2 = res.reshape((cv_Homography.shape))
        # # res2 = res.reshape((cv_image.shape))
        # # cv2.imshow('res2',res2)

#### converting to binary image
        # cv_Homography = cv2.cvtColor(cv_Homography, cv2.COLOR_RGB2GRAY)
        # ret, cv_Homography = cv2.threshold(cv_Homography, binary_threshold, 255, cv2.THRESH_BINARY)

        cv2.namedWindow('cv_image', cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow('cv_image', 0, 250)
        cv2.namedWindow('Homography', cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow('Homography', 500, 250)
        # cv2.namedWindow('res2', cv2.WINDOW_AUTOSIZE)
        # cv2.moveWindow('res2', 1000, 250)

        # showing calibrated iamge and Bird's eye view image
        if self.showing_images == "on":
            cv2.imshow('cv_image', cv_image), cv2.waitKey(1)
            cv2.imshow('Homography', cv_Homography), cv2.waitKey(1)

            # cv2.imshow('res2',res2), cv2.waitKey(1)
            # cv2.imshow('Downsampling', small_img), cv2.waitKey(1)

        # publishing calbrated and Bird's eye view as compressed image
        if self.selecting_pub_image == "compressed":
            msg_calibration = CompressedImage()
            msg_calibration.header.stamp = rospy.Time.now()
            msg_calibration.format = "jpeg"
            msg_calibration.data = np.array(cv2.imencode('.jpg', cv_origin)[1]).tostring()
            self._pub1.publish(msg_calibration)

            msg_homography = CompressedImage()
            msg_homography.header.stamp = rospy.Time.now()
            msg_homography.format = "jpeg"
            msg_homography.data = np.array(cv2.imencode('.jpg', cv_Homography)[1]).tostring()
            self._pub2.publish(msg_homography)

        # publishing calbrated and Bird's eye view as raw image
        elif self.selecting_pub_image == "raw":
            self._pub3.publish(self.bridge.cv2_to_imgmsg(cv_origin, "bgr8"))
            self._pub4.publish(self.bridge.cv2_to_imgmsg(cv_Homography, "bgr8"))
            # self._pub4.publish(self.bridge.cv2_to_imgmsg(cv_Homography, "mono8"))


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_to_ground')
    node = ImageTransform()
    node.main()
