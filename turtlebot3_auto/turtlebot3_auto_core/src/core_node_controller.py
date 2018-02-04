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

# Author: Ryu Woon Jung (Leon), [AuTURBO] Kihoon Kim

import rospy, roslaunch
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image, CompressedImage

from enum import Enum

class CoreNodeController():
    def __init__(self):
        self.sub_mode_control = rospy.Subscriber('/core/mode_decider/mode', UInt8, self.cbDecideMode, queue_size=1)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.launch_camera = roslaunch.scriptapi.ROSLaunch()
        self.launch_camera = roslaunch.parent.ROSLaunchParent(uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_camera/launch/turtlebot3_auto_camera_calibration.launch"])

        self.launch_detect_lane = roslaunch.scriptapi.ROSLaunch()
        self.launch_detect_lane = roslaunch.parent.ROSLaunchParent(uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_detect/launch/turtlebot3_auto_detect_lane.launch"])

        self.launch_control_lane = roslaunch.scriptapi.ROSLaunch()
        self.launch_control_lane = roslaunch.parent.ROSLaunchParent(uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_control/launch/turtlebot3_auto_control_lane.launch"])     


        self.Mode = Enum('Mode', 'idle lane_following traffic_light parking_lot level_crossing tunnel')

        self.mode_current = self.Mode.idle.value
        self.mode_previous = self.Mode.idle.value

        self.fnControlNode()

    def cbDecideMode(self, mode_msg):
        self.mode_current = mode_msg.data

        if self.mode_current != self.mode_previous:
            self.fnControlNode()

        self.mode_previous = self.mode_current

    def fnControlNode(self):
        if self.mode_current == self.Mode.idle.value:
            # pass
            self.mode_current = self.Mode.lane_following.value

        if self.mode_current == self.Mode.lane_following.value:
            if self.mode_previous == self.Mode.idle.value:
                self.launch_camera.start()
                self.launch_detect_lane.start()
                self.launch_control_lane.start()
            elif self.mode_previous == self.Mode.parking.value:
                pass
            elif self.mode_previous == self.Mode.level_crossing.value:
                pass
            elif self.mode_previous == self.Mode.tunnel.value:
                pass

        elif self.mode_current == self.Mode.parking.value:
            if self.mode_previous == self.Mode.lane_following.value:
                self.launch_detect_lane.shutdown()
                self.launch_control_lane.shutdown()
            elif self.mode_previous == self.Mode.level_crossing.value:
                pass
            elif self.mode_previous == self.Mode.tunnel.value:
                pass

        elif self.mode_current == self.Mode.level_crossing.value:
            if self.mode_previous == self.Mode.lane_following.value:
                self.launch_detect_lane.shutdown()
                self.launch_control_lane.shutdown()
            elif self.mode_previous == self.Mode.parking.value:
                pass
            elif self.mode_previous == self.Mode.tunnel.value:
                pass
        
        elif self.mode_current == self.Mode.tunnel.value:
            if self.mode_previous == self.Mode.lane_following.value:
                self.launch_detect_lane.shutdown()
                self.launch_control_lane.shutdown()
            elif self.mode_previous == self.Mode.parking.value:
                pass
            elif self.mode_previous == self.Mode.level_crossing.value:
                pass

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('core_node_controller')
    node = CoreNodeController()
    node.main()
