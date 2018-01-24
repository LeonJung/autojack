#!/usr/bin/env python

#Special thanks to Roger Sacchelli (https://github.com/rogersacchelli/Lanes-Detection-with-OpenCV)

import rospy
import cv2
import numpy as np
from sklearn.cluster import KMeans
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import tf


from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt

import math


def callback(x):
    pass

class DetectLane():
    def __init__(self):
        self.showing_plot_track = "off"
        self.showing_images = "off" # you can choose showing images or not by "on", "off"

        self.showing_final_image = "on"
        self.selecting_sub_image = "compressed" # you can choose image type "compressed", "raw"
        self.selecting_pub_image = "raw" # you can choose image type "compressed", "raw"

        if self.selecting_sub_image == "compressed":
            self._sub = rospy.Subscriber('/image_birdseye_compressed', CompressedImage, self.callback, queue_size = 1)
        else:
            self._sub = rospy.Subscriber('/image_birdseye', Image, self.callback, queue_size = 1)

        self._sub2 = rospy.Subscriber('/odom', Odometry, self.callback2, queue_size=1)

        # There are 4 Publishers
        # pub1 : calibrated image as compressed image
        # pub2 : Bird's eye view image as compressed image
        # pub3 : calibrated image as raw image
        # pub4 : Bird's eye view image as raw image
        # if you do not want to publish some topic, delete pub definition in __init__ function and publishing process in callback function
        if self.selecting_pub_image == "compressed":
            self._pub1 = rospy.Publisher('/image_lanes_compressed', CompressedImage, queue_size = 1)
        elif self.selecting_pub_image == "raw":
            self._pub2 = rospy.Publisher('/image_lanes', Image, queue_size = 1)

        self._pub3 = rospy.Publisher('/lane/desired_center', Float64, queue_size = 1)
        # self._pub4 = rospy.Publisher('/lane/off_center', Float64, queue_size = 1)

        self.cvBridge = CvBridge()

        self.counter = 0

        self.step = 4
        self.pos_err = 10.

        self.window_width = 1000.
        self.window_height = 600.

        self.screen_start_from_wheel_real_dist = 123.

        self.d_ppx = 29. / 57.
        self.d_ppy = 29. / 90.

        self.x_t = [0, 0, 0, 0]
        self.y_t = np.subtract(np.arange(0, self.window_height, (self.window_height / self.step)), 1)

        self.Hue_l_white = 7
        self.Hue_h_white = 84
        self.Saturation_l_white = 7
        self.Saturation_h_white = 58
        self.Lightness_l_white = 112
        self.Lightness_h_white = 255

        self.Hue_l_yellow = 30
        self.Hue_h_yellow = 115
        self.Saturation_l_yellow = 79
        self.Saturation_h_yellow = 255
        self.Lightness_l_yellow = 130
        self.Lightness_h_yellow = 255

    def callback2(self, odom_msg):
        self.now_pos_x = odom_msg.pose.pose.position.y * 1000.
        self.now_pos_y = odom_msg.pose.pose.position.x * 1000.

        (self.now_roll, self.now_pitch, self.now_yaw) = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])

    def callback(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        # if self.counter % 3 != 0:
        #     self.counter += 1
        #     return

        if self.selecting_sub_image == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image22 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.selecting_sub_image == "raw":
            cv_image22 = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        ## New method
        # TODO : processing speed
        # TODO : should this get image_to_ground image? or plain image? if it calculates based on plain image, the brightness varies on so much stuffs around

        clip_hist_percent = 1.0
        
        hist_size = 256
        alpha = 0
        beta = 0
        min_gray = 0
        max_gray = 0

        #gray = np.zeros([240, 320], dtype=np.uint8)

        gray = cv2.cvtColor(cv_image22, cv2.COLOR_BGR2GRAY)

        if clip_hist_percent == 0.0:
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray)
        else:
        # cv2.calcHist(images, channels, mask, histSize, ranges[, hist[, accumulate]])
            hist = cv2.calcHist([gray], [0], None, [hist_size], [0, hist_size])
            # hist, bin_edges = np.histogram(gray, normed=True)

            # plt.plot(hist)
            # plt.xlim([0,256])
            # plt.show()

        accumulator = np.zeros((hist_size))
        
        accumulator[0] = hist[0]

        for i in range(0, hist_size):
            accumulator[i] = accumulator[i - 1] + hist[i]

        max = accumulator[hist_size - 1]

        # print(max)

        clip_hist_percent *= (max / 100.)
        clip_hist_percent /= 2.

        min_gray = 0
        while accumulator[min_gray] < clip_hist_percent:
            min_gray += 1
        
        max_gray = hist_size - 1
        while accumulator[max_gray] >= (max - clip_hist_percent):
            max_gray -= 1

        input_range = max_gray - min_gray

        alpha = (hist_size - 1) / input_range
        beta = -min_gray * alpha

        cv_image = cv2.convertScaleAbs(cv_image22, -1, alpha, beta)












        # copy original image to use in lane finding process
        cv_white_lane = np.copy(cv_image)
        cv_yellow_lane = np.copy(cv_image)

        # find White and Yellow Lanes
        white_fraction, cv_white_lane = self.maskWhiteLane(cv_white_lane)
        yellow_fraction, cv_yellow_lane = self.maskYellowLane(cv_yellow_lane)

        # Bitwise-OR mask to sum up two lane images
        cv_lanes = cv2.bitwise_or(cv_white_lane, cv_yellow_lane, mask = None)

        if self.showing_images == "on":
            cv2.imshow('lanes', cv_lanes), cv2.waitKey(1)

        try:
            # desired_center, left_fit, right_fit = self.fit_from_lines(left_fit, right_fit, cv_lanes)
            left_fitx, left_fit = self.fit_from_lines2(left_fit, cv_white_lane, 'left')
            right_fitx, right_fit = self.fit_from_lines2(right_fit, cv_yellow_lane, 'right')

            mov_avg_left = np.append(mov_avg_left,np.array([left_fit]), axis=0)
            mov_avg_right = np.append(mov_avg_right,np.array([right_fit]), axis=0)

        except:
            left_fitx, left_fit = self.sliding_windown2(cv_white_lane, 'left')
            right_fitx, right_fit = self.sliding_windown2(cv_yellow_lane, 'right')

            mov_avg_left = np.array([left_fit])
            mov_avg_right = np.array([right_fit])

        MOV_AVG_LENGTH = 5

        left_fit = np.array([np.mean(mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
                             np.mean(mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
                             np.mean(mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])])
        right_fit = np.array([np.mean(mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
                             np.mean(mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
                             np.mean(mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])])

        if mov_avg_left.shape[0] > 1000:
            mov_avg_left = mov_avg_left[0:MOV_AVG_LENGTH]
        if mov_avg_right.shape[0] > 1000:
            mov_avg_right = mov_avg_right[0:MOV_AVG_LENGTH]



        centerx = np.mean([left_fitx, right_fitx], axis=0)


        # t_fit = time.time() - t_fit0

        # t_draw0 = time.time()
        if self.showing_final_image == "on":
            final = self.draw_lines(cv_image, cv_lanes, left_fit, right_fit)
        # final = self.draw_lines(cv_image, cv_lanes, left_fit, right_fit, perspective=[src,dst])


######################################################## test

        x_r = self.now_pos_x
        y_r = self.now_pos_y
        theta_r = self.now_yaw

        # x_t = 0.
        # y_t = 380.



        # theta_a = (math.atan2((y_t - y_r), (x_t - x_r)))
        # theta_p = theta_a + theta_r

        # print(math.degrees(theta_a))
        # print(math.degrees(self.now_yaw))
        # print(math.degrees(theta_p))

        # rospy.loginfo("%d", (int)((x_t - x_r) ** 2 + (y_t - y_r) ** 2))

        # x_d = -math.sqrt((x_t - x_r) ** 2 + (y_t - y_r) ** 2) * math.cos(theta_p)
        # y_d = math.sqrt((x_t - x_r) ** 2 + (y_t - y_r) ** 2) * math.sin(theta_p)



        # x_reald = x_d / self.d_ppx + self.window_width / 2.
        # y_reald = self.window_height - (y_d - self.screen_start_from_wheel_real_dist) / self.d_ppy

        # cv2.circle(final,((int)(x_reald), (int)(y_reald)), 21, (0,0,255), -1)

##########################################################


        # self.x_t = np.concatenate((self.x_t, [5.0]), axis=1)

        # if (((x_r >= (self.x_t[0] - self.pos_err)) || (x_r < (self.x_t[0] + self.pos_err))) && ((y_r >= (self.y_t[0] - self.pos_err)) || (y_r < (self.y_t[0] + self.pos_err)))):
        #     self.x_t = np.delete(self.x_t, 0)
        #     self.y_t = np.delete(self.y_t, 0)

        #     self.y_t = np.append(self.y_t, self.y_t[self.step - 2] + (self.window_height / self.step))
        #     self.x_t = np.append(self.x_t, centerx[(int)self.y_t[self.step - 1]])

            

        #     # publish(x_t, y_t)

        # print(self.y_t)


            

        # for i in range(0, step):


        # pos_1_y_reald = self.window_height / step




        # sampling_rate = 5 #Hz

        # self.counter %= sampling_rate
        # if self.counter == 0:
        #     self.center_samples = np.zeros([sampling_rate, centerx.shape[0]])
        
        # self.center_samples[self.counter, :centerx.shape[0]] = centerx

        # if self.counter == (sampling_rate - 1):
        #     self.center_samples_mean = np.mean(self.center_samples, axis = 0)
        #     # print(self.center_samples)
        #     # print(self.center_samples_mean)
                       

        #     self.center_samples_var = np.subtract(self.center_samples, self.center_samples_mean)
        #     # print(self.center_samples_var)

        #     # np.argmin(self.center_samples_mean)
        #     # print( np.argmin(self.center_samples_var, axis=0))

        #     self.centerx_new = 

        self.counter += 1

        


            # self.center_samples = np.copy(centerx)
# >>> import numpy as np
# >>> a = np.arange(9)
# >>> a = a.reshape((3, 3))
# >>> b = np.zeros((5, 5))
# >>> b[1:4, 1:4] = a
# >>> b
# array([[ 0.,  0.,  0.,  0.,  0.],
#        [ 0.,  0.,  1.,  2.,  0.],
#        [ 0.,  3.,  4.,  5.,  0.],
#        [ 0.,  6.,  7.,  8.,  0.],
#        [ 0.,  0.,  0.,  0.,  0.]])

# >>> b[1:4,1:4] = a + 1  # If you really meant `[1, 2, ..., 9]`
# >>> b
# array([[ 0.,  0.,  0.,  0.,  0.],
#        [ 0.,  1.,  2.,  3.,  0.],
#        [ 0.,  4.,  5.,  6.,  0.],
#        [ 0.,  7.,  8.,  9.,  0.],
#        [ 0.,  0.,  0.,  0.,  0.]])

        # center_samples

        # print(self.center_samples)
        # print(centerx)

        # print(centerx.shape)
        # rospy.loginfo("%d", centerx.shape(0))

        # centerx = np.mean([left_fitx, right_fitx], axis=0)

                # self.counter += 1
        # if self.counter % 3 != 0:
        #     return

        x_reald = 1
        y_reald = 1

        x_d_new = self.d_ppx * (x_reald - self.window_width / 2.)
        y_d_new = -self.d_ppy * (y_reald - self.window_height) + self.screen_start_from_wheel_real_dist

        theta_p_new = -math.acos(x_d_new / math.sqrt(x_d_new ** 2 + y_d_new ** 2))
        theta_a_new = theta_p_new - theta_r

        # rospy.loginfo("x_d_new : %f", x_d_new)
        # rospy.loginfo("theta p : %f a : %f", theta_p_new, theta_a_new)

        x_t_new = x_r - math.sqrt((x_d_new ** 2 + y_d_new ** 2) / (math.tan(theta_a_new) ** 2 + 1))
        y_t_new = y_r + (x_t_new - x_r) * math.tan(theta_a_new)

        # rospy.loginfo("now_pos_x : %d  now_pos_y : %d", self.now_pos_x, self.now_pos_y)

        # rospy.loginfo("%d %d", x_t_new, y_t_new)




















        # cv2.circle(final,((int)(x_reald), (int)(y_reald)), 21, (0,0,255), -1)

        # rospy.loginfo("%d %d", x_reald, y_reald)

        # cv2.circle(final,((int)(x_reald), (int)(y_reald)), 21, (0,0,255), -1)


        # x = 500
        # # y = 0

        # y = 380. # real
        # screen_end_from_wheel_real_dist = 320.
        # real_dist_per_pix_y = 90. / 29.
        
        # # print(self.now_pos_x)

        # real_point_x = x
        # in_picture_point_y = real_dist_per_pix_y * -(y - screen_end_from_wheel_real_dist - self.now_pos_x)

        # print(self.now_pos_x)
        # print(in_picture_point_y)

        # cv2.circle(final,(real_point_x, (int)(in_picture_point_y)), 21, (0,0,255), -1)


        # cv2.circle(final,(real_point_x, (int)((29. ))), 21, (0,0,255), -1)















        # print(y_reald)


        # cv2.circle(final,((int)(x_reald), (int)(y_reald)), 21, (0,0,255), -1)


        # theta_a = math.acos(x_d / math.sqrt((x_d ** 2) + (y_d ** 2)))



        # l = math.sin(math.radians(30.0))

        # l = math.sin(math.pi / 2.)

        # print(l)
        # print(self.now_yaw)


        # cv2.circle(final,((int)(x_d_init), (int)(y_d_init)), 21, (0,0,255), -1)



















#######################################################



        # x = 500
        # # y = 0

        # y = 380. # real
        # screen_end_from_wheel_real_dist = 320.
        # real_dist_per_pix_y = 90. / 29.
        
        # # print(self.now_pos_x)

        # real_point_x = x
        # in_picture_point_y = real_dist_per_pix_y * -(y - screen_end_from_wheel_real_dist - self.now_pos_x)

        # print(self.now_pos_x)
        # print(in_picture_point_y)

        # cv2.circle(final,(real_point_x, (int)(in_picture_point_y)), 21, (0,0,255), -1)


        # cv2.circle(final,(real_point_x, (int)((29. ))), 21, (0,0,255), -1)



#######################################################






















        if self.showing_final_image == "on":
            cv2.imshow('final', final), cv2.waitKey(1)

        # publishing calbrated and Bird's eye view as compressed image
        if self.selecting_pub_image == "compressed":
            # msg_final_img = CompressedImage()
            # msg_final_img.header.stamp = rospy.Time.now()
            # msg_final_img.format = "jpeg"
            # msg_final_img.data = np.array(cv2.imencode('.jpg', final)[1]).tostring()
            # self._pub1.publish(msg_final_img)

            msg_desired_center = Float64()
            msg_desired_center.data = desired_center.item(450)
            # msg_desired_center.data = desired_center.item(300)

            # msg_off_center = Float64()
            # msg_off_center.data = off_center

            self._pub3.publish(msg_desired_center)
            # self._pub4.publish(msg_off_center)

            # msg_homography = CompressedImage()
            # msg_homography.header.stamp = rospy.Time.now()
            # msg_homography.format = "jpeg"
            # msg_homography.data = np.array(cv2.imencode('.jpg', cv_Homography)[1]).tostring()
            # self._pub2.publish(msg_homography)

        # publishing calbrated and Bird's eye view as raw image
        elif self.selecting_pub_image == "raw":
            # self._pub2.publish(self.cvBridge.cv2_to_imgmsg(final, "bgr8"))

            msg_desired_center = Float64()
            # msg_desired_center.data = desired_center.item(300)

            # msg_off_center = Float64()
            # msg_off_center.data = off_center

            # self._pub3.publish(msg_desired_center)
            # self._pub4.publish(msg_off_center)

            # self._pub4.publish(self.bridge.cv2_to_imgmsg(cv_Homography, "bgr8"))
            # self._pub4.publish(self.bridge.cv2_to_imgmsg(cv_Homography, "mono8"))

    def maskWhiteLane(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.Hue_l_white
        Hue_h = self.Hue_h_white
        Saturation_l = self.Saturation_l_white
        Saturation_h = self.Saturation_h_white
        Lightness_l = self.Lightness_l_white
        Lightness_h = self.Lightness_h_white

        if self.showing_images == "on":
            cv2.namedWindow('mask_white')
            cv2.createTrackbar('Hue_l', 'mask_white', Hue_l, 179, callback)
            cv2.createTrackbar('Hue_h', 'mask_white', Hue_h, 180, callback)
            cv2.createTrackbar('Saturation_l', 'mask_white', Saturation_l, 254, callback)
            cv2.createTrackbar('Saturation_h', 'mask_white', Saturation_h, 255, callback)
            cv2.createTrackbar('Lightness_l', 'mask_white', Lightness_l, 254, callback)
            cv2.createTrackbar('Lightness_h', 'mask_white', Lightness_h, 255, callback)

            # getting homography variables from trackbar
            Hue_l = cv2.getTrackbarPos('Hue_l', 'mask_white')
            Hue_h = cv2.getTrackbarPos('Hue_h', 'mask_white')
            Saturation_l = cv2.getTrackbarPos('Saturation_l', 'mask_white')
            Saturation_h = cv2.getTrackbarPos('Saturation_h', 'mask_white')
            Lightness_l = cv2.getTrackbarPos('Lightness_l', 'mask_white')
            Lightness_h = cv2.getTrackbarPos('Lightness_h', 'mask_white')

        # define range of white color in HSV
        lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_white = np.array([Hue_h, Saturation_h, Lightness_h])


        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        # cv2.imshow('frame_white',image), cv2.waitKey(1)
        if self.showing_images == "on":
            cv2.imshow('mask_white',mask), cv2.waitKey(1)
        # cv2.imshow('res_white',res), cv2.waitKey(1)

        fraction_num = np.count_nonzero(mask)

        if fraction_num > 35000:
            if self.Lightness_l_white < 250:
                self.Lightness_l_white += 1
        elif fraction_num < 5000:
            if self.Lightness_l_white > 10:
                self.Lightness_l_white -= 1
        
        # print(self.Lightness_l_white)
        # print(fraction_num)

        return fraction_num, mask

    def maskYellowLane(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.Hue_l_yellow
        Hue_h = self.Hue_h_yellow
        Saturation_l = self.Saturation_l_yellow
        Saturation_h = self.Saturation_h_yellow
        Lightness_l = self.Lightness_l_yellow
        Lightness_h = self.Lightness_h_yellow

        if self.showing_images == "on":
            cv2.namedWindow('mask_yellow')
            cv2.createTrackbar('Hue_l', 'mask_yellow', Hue_l, 179, callback)
            cv2.createTrackbar('Hue_h', 'mask_yellow', Hue_h, 180, callback)
            cv2.createTrackbar('Saturation_l', 'mask_yellow', Saturation_l, 254, callback)
            cv2.createTrackbar('Saturation_h', 'mask_yellow', Saturation_h, 255, callback)
            cv2.createTrackbar('Lightness_l', 'mask_yellow', Lightness_l, 254, callback)
            cv2.createTrackbar('Lightness_h', 'mask_yellow', Lightness_h, 255, callback)

            # getting homography variables from trackbar
            Hue_l = cv2.getTrackbarPos('Hue_l', 'mask_yellow')
            Hue_h = cv2.getTrackbarPos('Hue_h', 'mask_yellow')
            Saturation_l = cv2.getTrackbarPos('Saturation_l', 'mask_yellow')
            Saturation_h = cv2.getTrackbarPos('Saturation_h', 'mask_yellow')
            Lightness_l = cv2.getTrackbarPos('Lightness_l', 'mask_yellow')
            Lightness_h = cv2.getTrackbarPos('Lightness_h', 'mask_yellow')

        # define range of yellow color in HSV
        lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        # cv2.imshow('frame_yellow',image), cv2.waitKey(1)
        if self.showing_images == "on":
            cv2.imshow('mask_yellow',mask), cv2.waitKey(1)
        # cv2.imshow('res_yellow',res), cv2.waitKey(1)


        fraction_num = np.count_nonzero(mask)

        # if fraction_num > 40000:
        #     Lightness_l_yellow += 5
        # elif fraction_num < 40000:
        #     Lightness_l_yellow -= 5
        # print(fraction_num)

        return fraction_num, mask

    def fit_from_lines(self, left_fit, right_fit, img_w):
        # Assume you now have a new warped binary image
        # from the next frame of video (also called "binary_warped")
        # It's now much easier to find line pixels!
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] - margin)) & (
        nonzerox < (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] + margin)))
        right_lane_inds = (
        (nonzerox > (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] - margin)) & (
        nonzerox < (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] + margin)))

        # Again, extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
            
        centerx = np.mean([left_fitx, right_fitx], axis=0)
        # print(centerx.item(300))

        # for i, line in enumerate(text.split('\n')):
        #     i = 50 + 20 * i
        #     cv2.putText(result, line, (0,i), cv2.FONT_HERSHEY_DUPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)

        if self.showing_plot_track == "on":
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
            plt.imshow(out_img)
            plt.plot(left_fitx, ploty, color='yellow')
            plt.plot(right_fitx, ploty, color='yellow')
            plt.xlim(0, 1280)
            plt.ylim(720, 0)
            plt.draw()
            plt.pause(0.00000000001)
            plt.ion()
            plt.clf()
            plt.show()

        return centerx, left_fit, right_fit

    def fit_from_lines2(self, left_fit, img_w, left_or_right):
        # Assume you now have a new warped binary image
        # from the next frame of video (also called "binary_warped")
        # It's now much easier to find line pixels!
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] - margin)) & (
        nonzerox < (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] + margin)))

        # Again, extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            
        # centerx = np.mean([left_fitx, right_fitx], axis=0)
        # print(centerx.item(300))

        # for i, line in enumerate(text.split('\n')):
        #     i = 50 + 20 * i
        #     cv2.putText(result, line, (0,i), cv2.FONT_HERSHEY_DUPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)

        if self.showing_plot_track == "on":
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            plt.imshow(out_img)
            plt.plot(left_fitx, ploty, color='yellow')
            plt.xlim(0, 1280)
            plt.ylim(720, 0)
            plt.draw()
            plt.pause(0.00000000001)
            plt.ion()
            plt.clf()
            plt.show()

        return left_fitx, left_fit

    def sliding_windown2(self, img_w, left_or_right):
        histogram = np.sum(img_w[img_w.shape[0] / 2:, :], axis=0)

        # Create an output image to draw on and visualize the result
        out_img = np.dstack((img_w, img_w, img_w)) * 255

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0] / 2)

        if left_or_right == 'left':
            leftx_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            leftx_base = np.argmax(histogram[midpoint:]) + midpoint

        # rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 20#9
        # Set height of windows
        window_height = np.int(img_w.shape[0] / nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        # rightx_current = rightx_base
        # Set the width of the windows +/- margin
        margin = 50#100
        # Set minimum number of pixels found to recenter window
        minpix = 50
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        # right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            # win_xright_low = rightx_current - margin
            # win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            # try:
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            # cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
            
            # except:
            #     rospy.loginfo("%d %d %d %d", win_xleft_low, win_y_low, win_xleft_high, win_y_high)
            #     rospy.loginfo("%d %d %d %d", win_xright_low, win_y_low, win_xright_high, win_y_high)
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                nonzerox < win_xleft_high)).nonzero()[0]
            # good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
            #     nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            # right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            # if len(good_right_inds) > minpix:
            #     rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        # right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        # rightx = nonzerox[right_lane_inds]
        # righty = nonzeroy[right_lane_inds]

        # Fit a second order polynomial to each
        try:
            left_fit = np.polyfit(lefty, leftx, 2)
            self.left_fit_bef = left_fit
        except:
            left_fit = self.left_fit_bef
        # right_fit = np.polyfit(righty, rightx, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        # right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        # centerx = np.mean([left_fitx, right_fitx], axis=0)
        # print(centerx.item(300))

        if self.showing_plot_track == "on":
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
            plt.imshow(out_img)
            plt.plot(left_fitx, ploty, color='yellow')
            # plt.plot(right_fitx, ploty, color='yellow')
            plt.xlim(0, 1000)
            plt.ylim(600, 0)
            plt.draw()
            plt.pause(0.00000000001)
            plt.ion()
            plt.clf()
            plt.show()

        return left_fitx, left_fit

    def sliding_windown(self, img_w):
        histogram = np.sum(img_w[img_w.shape[0] / 2:, :], axis=0)

        # Create an output image to draw on and visualize the result
        out_img = np.dstack((img_w, img_w, img_w)) * 255

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0] / 2)

        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 20#9
        # Set height of windows
        window_height = np.int(img_w.shape[0] / nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 50
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            # try:
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
            
            # except:
            #     rospy.loginfo("%d %d %d %d", win_xleft_low, win_y_low, win_xleft_high, win_y_high)
            #     rospy.loginfo("%d %d %d %d", win_xright_low, win_y_low, win_xright_high, win_y_high)
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        centerx = np.mean([left_fitx, right_fitx], axis=0)
        # print(centerx.item(300))

        if self.showing_plot_track == "on":
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
            plt.imshow(out_img)
            plt.plot(left_fitx, ploty, color='yellow')
            plt.plot(right_fitx, ploty, color='yellow')
            plt.xlim(0, 1000)
            plt.ylim(600, 0)
            plt.draw()
            plt.pause(0.00000000001)
            plt.ion()
            plt.clf()
            plt.show()

        return centerx, left_fit, right_fit

    def draw_lines(self, img, img_w, left_fit, right_fit):
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(img_w).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        #color_warp_center = np.dstack((warp_zero, warp_zero, warp_zero))

        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])

        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        centerx = np.mean([left_fitx, right_fitx], axis=0)

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

        # Draw the lane onto the warped blank image
        #cv2.fillPoly(color_warp_center, np.int_([pts]), (0, 255, 0))
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        if self.showing_images == "on":
            cv2.imshow('color_warp', color_warp), cv2.waitKey(1)

        # # Warp the blank back to original image space using inverse perspective matrix (Minv)
        # newwarp = warp(color_warp, perspective[1], perspective[0])
        # # Combine the result with the original image
        result = cv2.addWeighted(img, 1, color_warp, 0.2, 0)

        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))
        cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=25)
        cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=25)
        cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)
        # newwarp_lines = warp(color_warp_lines, perspective[1], perspective[0])

        if self.showing_images == "on":
            cv2.imshow('color_warp_lines', color_warp_lines), cv2.waitKey(1)

        result = cv2.addWeighted(result, 1, color_warp_lines, 1, 0)

        # ----- Radius Calculation ------ #

        img_height = img.shape[0]
        y_eval = img_height

        ym_per_pix = 0.029 / 90.  # meters per pixel in y dimension
        xm_per_pix = 0.029 / 57.  # meters per pixel in x dimension

        ploty = np.linspace(0, img_height - 1, img_height)
        # Fit new polynomials to x,y in world space
        left_fit_cr = np.polyfit(ploty * ym_per_pix, left_fitx * xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty * ym_per_pix, right_fitx * xm_per_pix, 2)

        # Calculate the new radii of curvature
        left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * left_fit_cr[0])

        right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * right_fit_cr[0])

        # radius = round((float(left_curverad) + float(right_curverad))/2.,2)

        rospy.loginfo("left curverad : %f right curverad : %f", left_curverad, right_curverad)

        # ----- Off Center Calculation ------ #

        # lane_width = (right_fit[2] - left_fit[2]) * xm_per_pix
        # center = (right_fit[2] - left_fit[2]) / 2
        # off_left = (center - left_fit[2]) * xm_per_pix
        # off_right = -(right_fit[2] - center) * xm_per_pix

        # off_center = round((center - img.shape[0] / 2.) * xm_per_pix,2)

        # --- Print text on screen ------ #
        #if radius < 5000.0:
        # text = "radius = %s [m]\noffcenter = %s [m]" % (str(radius), str(off_center))
        #text = "radius = -- [m]\noffcenter = %s [m]" % (str(off_center))

        # rospy.loginfo("%s", text)

        # for i, line in enumerate(text.split('\n')):
        #     i = 50 + 20 * i
        #     cv2.putText(result, line, (0,i), cv2.FONT_HERSHEY_DUPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
        return result


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_lane')
    node = DetectLane()
    node.main()
