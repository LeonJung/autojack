#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sklearn.cluster import KMeans
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import matplotlib.pyplot as plt


def callback(x):
    pass

class DetectLane():
    def __init__(self):
        self.showing_images = "off" # you can choose showing images or not by "on", "off"
        self.selecting_sub_image = "compressed" # you can choose image type "compressed", "raw"
        self.selecting_pub_image = "compressed" # you can choose image type "compressed", "raw"

        if self.selecting_sub_image == "compressed":
            self._sub = rospy.Subscriber('/image_birdseye_compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub = rospy.Subscriber('/image_birdseye', Image, self.callback, queue_size=1)

        # There are 4 Publishers
        # pub1 : calibrated image as compressed image
        # pub2 : Bird's eye view image as compressed image
        # pub3 : calibrated image as raw image
        # pub4 : Bird's eye view image as raw image
        # if you do not want to publish some topic, delete pub definition in __init__ function and publishing process in callback function
        if self.selecting_pub_image == "compressed":
            self._pub1 = rospy.Publisher('/image_lanes_compressed', CompressedImage, queue_size=1)
            # self._pub2 = rospy.Publisher('/image_birdseye_compressed', CompressedImage, queue_size=1)
        elif self.selecting_pub_image == "raw":
            self._pub3 = rospy.Publisher('/image_lanes', Image, queue_size=1)
            # self._pub4 = rospy.Publisher('/image_birdseye', Image, queue_size=1)

        self.cvBridge = CvBridge()

        self.counter = 0
        
    def callback(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        # if self.counter % 3 != 0:
        #     self.counter += 1
        #     return

        if self.selecting_sub_image == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.selecting_sub_image == "raw":
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        # copy original image to use in lane finding process
        cv_white_lane = np.copy(cv_image)
        cv_yellow_lane = np.copy(cv_image)

        cv_white_lane = self.findWhiteLane(cv_white_lane)
        cv_yellow_lane = self.findYellowLane(cv_yellow_lane)

        # cv2.imshow('cv_white_lane', cv_white_lane), cv2.waitKey(1)
        # cv2.imshow('cv_yellow_lane', cv_yellow_lane), cv2.waitKey(1)

        # Bitwise-OR mask and original image
        res = cv2.bitwise_or(cv_white_lane, cv_yellow_lane, mask = None)

        cv2.imshow('lanes', res), cv2.waitKey(1)

        try:
            left_fit, right_fit = self.fit_from_lines(left_fit, right_fit, res)

            mov_avg_left = np.append(mov_avg_left,np.array([left_fit]), axis=0)
            mov_avg_right = np.append(mov_avg_right,np.array([right_fit]), axis=0)

        except:
            left_fit, right_fit = self.sliding_windown(res)

            mov_avg_left = np.array([left_fit])
            mov_avg_right = np.array([right_fit])

        MOV_AVG_LENGTH = 5

        left_fit = np.array([np.mean(mov_avg_left[::-1][:,0][0:MOV_AVG_LENGTH]),
                             np.mean(mov_avg_left[::-1][:,1][0:MOV_AVG_LENGTH]),
                             np.mean(mov_avg_left[::-1][:,2][0:MOV_AVG_LENGTH])])
        right_fit = np.array([np.mean(mov_avg_right[::-1][:,0][0:MOV_AVG_LENGTH]),
                             np.mean(mov_avg_right[::-1][:,1][0:MOV_AVG_LENGTH]),
                             np.mean(mov_avg_right[::-1][:,2][0:MOV_AVG_LENGTH])])

        if mov_avg_left.shape[0] > 1000:
            mov_avg_left = mov_avg_left[0:MOV_AVG_LENGTH]
        if mov_avg_right.shape[0] > 1000:
            mov_avg_right = mov_avg_right[0:MOV_AVG_LENGTH]


        # t_fit = time.time() - t_fit0

        # t_draw0 = time.time()
        # final = self.draw_lines(cv_image, res, left_fit, right_fit)
        # final = self.draw_lines(cv_image, res, left_fit, right_fit, perspective=[src,dst])























        # publishing calbrated and Bird's eye view as compressed image
        if self.selecting_pub_image == "compressed":
            msg_calibration = CompressedImage()
            msg_calibration.header.stamp = rospy.Time.now()
            msg_calibration.format = "jpeg"
            msg_calibration.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
            self._pub1.publish(msg_calibration)

            # msg_homography = CompressedImage()
            # msg_homography.header.stamp = rospy.Time.now()
            # msg_homography.format = "jpeg"
            # msg_homography.data = np.array(cv2.imencode('.jpg', cv_Homography)[1]).tostring()
            # self._pub2.publish(msg_homography)

        # publishing calbrated and Bird's eye view as raw image
        elif self.selecting_pub_image == "raw":
            self._pub3.publish(self.cvBridge.cv2_to_imgmsg(cv_image, "bgr8"))
            # self._pub4.publish(self.bridge.cv2_to_imgmsg(cv_Homography, "bgr8"))
            # self._pub4.publish(self.bridge.cv2_to_imgmsg(cv_Homography, "mono8"))

    def findWhiteLane(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = 34
        Hue_h = 65
        Saturation_l = 13
        Saturation_h = 65
        Lightness_l = 0
        Lightness_h = 255

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
        cv2.imshow('mask_white',mask), cv2.waitKey(1)
        # cv2.imshow('res_white',res), cv2.waitKey(1)

        return mask

    def findYellowLane(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = 30
        Hue_h = 115
        Saturation_l = 80
        Saturation_h = 255
        Lightness_l = 80
        Lightness_h = 255

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
        cv2.imshow('mask_yellow',mask), cv2.waitKey(1)
        # cv2.imshow('res_yellow',res), cv2.waitKey(1)

        return mask

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

        return left_fit, right_fit

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
        nwindows = 9
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
            try:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
            
            except:
                rospy.loginfo("%d %d %d %d", win_xleft_low, win_y_low, win_xleft_high, win_y_high)
                rospy.loginfo("%d %d %d %d", win_xright_low, win_y_low, win_xright_high, win_y_high)
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
        # ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        # left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        # right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
        #
        # out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
        # plt.imshow(out_img)
        # plt.plot(left_fitx, ploty, color='yellow')
        # plt.plot(right_fitx, ploty, color='yellow')
        # plt.xlim(0, 1280)
        # plt.ylim(720, 0)

        return left_fit, right_fit

    # def draw_lines(self, img, img_w, left_fit, right_fit, perspective):
    def draw_lines(self, img, img_w, left_fit, right_fit):
        # # Create an image to draw the lines on
        # warp_zero = np.zeros_like(img_w).astype(np.uint8)
        # color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        # #color_warp_center = np.dstack((warp_zero, warp_zero, warp_zero))

        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])

        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        # # Recast the x and y points into usable format for cv2.fillPoly()
        # pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        # pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        # pts = np.hstack((pts_left, pts_right))

        # # Draw the lane onto the warped blank image
        # #cv2.fillPoly(color_warp_center, np.int_([pts]), (0, 255, 0))
        # cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # # Warp the blank back to original image space using inverse perspective matrix (Minv)
        # newwarp = warp(color_warp, perspective[1], perspective[0])
        # # Combine the result with the original image
        # result = cv2.addWeighted(img, 1, newwarp, 0.2, 0)

        # color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))
        # cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=25)
        # cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=25)
        # newwarp_lines = warp(color_warp_lines, perspective[1], perspective[0])

        # result = cv2.addWeighted(result, 1, newwarp_lines, 1, 0)

        # ----- Radius Calculation ------ #

        img_height = img.shape[0]
        y_eval = img_height

        ym_per_pix = 30 / 720.  # meters per pixel in y dimension
        xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

        ploty = np.linspace(0, img_height - 1, img_height)
        # Fit new polynomials to x,y in world space
        left_fit_cr = np.polyfit(ploty * ym_per_pix, left_fitx * xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty * ym_per_pix, right_fitx * xm_per_pix, 2)

        # Calculate the new radii of curvature
        left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * left_fit_cr[0])

        right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * right_fit_cr[0])

        radius = round((float(left_curverad) + float(right_curverad))/2.,2)

        # ----- Off Center Calculation ------ #

        lane_width = (right_fit[2] - left_fit[2]) * xm_per_pix
        center = (right_fit[2] - left_fit[2]) / 2
        off_left = (center - left_fit[2]) * xm_per_pix
        off_right = -(right_fit[2] - center) * xm_per_pix
        off_center = round((center - img.shape[0] / 2.) * xm_per_pix,2)

        # --- Print text on screen ------ #
        #if radius < 5000.0:
        text = "radius = %s [m]\noffcenter = %s [m]" % (str(radius), str(off_center))
        #text = "radius = -- [m]\noffcenter = %s [m]" % (str(off_center))

        rospy.loginfo("%s", text)

        # for i, line in enumerate(text.split('\n')):
        #     i = 50 + 20 * i
        #     cv2.putText(result, line, (0,i), cv2.FONT_HERSHEY_DUPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
        # return result


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_lane')
    node = DetectLane()
    node.main()
