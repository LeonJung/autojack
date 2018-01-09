#!/usr/bin/env python

import rospkg
import rospy
import yaml
import thread
import io

from picamera import PiCamera

from sensor_msgs.msg import CompressedImage
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

class RaspicamNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] node started" %(self.node_name))

		self.is_shutdown = False
		self.is_published = False

		self.stream = io.BytesIO()
		self.calibration_file_folder_path = "catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_settings/calibrations/camera_intrinsic/"
		self.frame_id = "/camera_optical_frame"

		# Messages
		self.msgCompressedImage = CompressedImage()
		
		self.pubImg = rospy.Publisher("~image/compressed", CompressedImage, queue_size = 1)		

		# Services
		self.resCameraInfo = SetCameraInfoResponse()

		self.srvSetCameraInfo = rospy.Service("~set_camera_info", SetCameraInfo, self.cbSetCameraInfo)

		# Settings
		self.framerate_high = self.setParam("~framerate_high", 30.0)
		self.framerate_low = self.setParam("~framerate_low", 15.0)
		self.res_w = self.setParam("~res_w", 640)
		self.res_h = self.setParam("~res_h", 480)

		## framerate default as high
		self.framerate = self.framerate_high


		rospy.loginfo("[%s] parameters initialized" %(self.node_name))

		# PiCamera
		self.camera = PiCamera()
		self.camera.framerate = self.framerate
		self.camera.resolution = (self.res_w, self.res_h)






	def setParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s" %(self.node_name, param_name, value))
		return value

	def captureImage(self):
		rospy.loginfo("[%s] capturing image..." %(self.node_name))
		while not self.is_shutdown and not rospy.is_shutdown():
			gen =  self.grabAndPublish(self.stream, self.pubImg)
			try:
				self.camera.capture_sequence(gen, 'jpeg', use_video_port=True, splitter_port=0)
			except StopIteration:
				pass
			print "updating framerate"
			self.camera.framerate = self.framerate
			self.update_framerate = False

		self.camera.close()
		rospy.loginfo("[%s] Capture Ended." %(self.node_name))

	def grabAndPublish(self, stream, pubImg):
		while not self.is_shutdown and not rospy.is_shutdown():
			yield stream
			# Construct image_msg
			# Grab image from stream
			stamp = rospy.Time.now()	
			stream.seek(0)
			stream_data = stream.getvalue()

			# Generate compressed image
			msgCompressedImage = CompressedImage()
			msgCompressedImage.format = "jpeg"
			msgCompressedImage.data = stream_data

			msgCompressedImage.header.stamp = stamp
			msgCompressedImage.header.frame_id = self.frame_id
			pubImg.publish(msgCompressedImage)
                        
			# Clear stream
			stream.seek(0)
			stream.truncate()
            
			if not self.is_published:
				rospy.loginfo("[%s] Published the first image." %(self.node_name))
				self.is_published = True

			rospy.sleep(rospy.Duration.from_sec(0.001))



	def cbSetCameraInfo(self, reqCameraInfo):
		rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
		file_name = self.calibration_file_folder_path + "default.yaml"
		self.resCameraInfo.success = self.saveCameraInfo(reqCameraInfo.camera_info, file_name)
		self.resCameraInfo.status_message = "Write to %s" %file_name
		return self.resCameraInfo

	def saveCameraInfo(self, msgCameraInfo, file_name):
		# Convert camera_info_msg and save to a yaml file
		rospy.loginfo("[saveCameraInfo] filename: %s" %(file_name))

		calib = {'image_width': msgCameraInfo.width,
			'image_height': msgCameraInfo.height,
			'camera_name': rospy.get_name().strip("/"),
			'distortion_model': msgCameraInfo.distortion_model,
			'distortion_coefficients': {'data': msgCameraInfo.D, 'rows':1, 'cols':5},
			'camera_matrix': {'data': msgCameraInfo.K, 'rows':3, 'coles':3},
			'rectification_matrix': {'data': msgCameraInfo.R, 'rows':3, 'cols':3},
			'projection_matrix': {'data': msgCameraInfo.P, 'ros':3, 'cols':4}}

		rospy.loginfo("[saveCameraInfo] calib %s" %(calib))


		try:
			f = open(file_name, 'w')
			yaml.safe_dump(calib, f)
			return True
		except IOError:
			return False

	def onShutdown(self):
		rospy.loginfo("[%s] node terminated" %(self.node_name))

if __name__ == '__main__':
	rospy.init_node('raspicam_node', anonymous = False)
	raspicam_node = RaspicamNode()
	rospy.on_shutdown(raspicam_node.onShutdown)
	thread.start_new_thread(raspicam_node.captureImage, ())
	rospy.spin()
