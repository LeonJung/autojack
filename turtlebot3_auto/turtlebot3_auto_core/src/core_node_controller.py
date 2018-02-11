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
        self.sub_mode_control = rospy.Subscriber('/core/decided_mode', UInt8, self.cbReceiveMode, queue_size=1)

        self.CurrentMode = Enum('CurrentMode', 'idle lane_following traffic_light parking_lot level_crossing tunnel')

        # subscribes : status returned
        self.sub_traffic_light_stamped = rospy.Subscriber('/detect/traffic_light_stamped', UInt8, self.cbTrafficLightStamped, queue_size=1)
        self.sub_parking_lot_stamped = rospy.Subscriber('/detect/parking_lot_stamped', UInt8, self.cbParkingLotStamped, queue_size=1)
        self.sub_level_crossing_stamped = rospy.Subscriber('/detect/level_crossing_stamped', UInt8, self.cbLevelCrossingStamped, queue_size=1)
        self.sub_tunnel_stamped = rospy.Subscriber('/detect/tunnel_stamped', UInt8, self.cbTunnelStamped, queue_size=1)


        # publishes orders
        self.pub_traffic_light_order = rospy.Publisher('/detect/traffic_light_order', UInt8, queue_size=1)
        self.pub_parking_lot_order = rospy.Publisher('/detect/parking_lot_order', UInt8, queue_size=1)
        self.pub_level_crossing_order = rospy.Publisher('/detect/level_crossing_order', UInt8, queue_size=1)
        self.pub_tunnel_order = rospy.Publisher('/detect/tunnel_order', UInt8, queue_size=1)

        self.pub_mode_return = rospy.Publisher('/core/returned_mode', UInt8, queue_size=1)

        self.StepOfTrafficLight = Enum('StepOfTrafficLight', 'searching_traffic_light traffic_light_detected watching_green_light yellow_light_detected watching yellow_light red_light_detected watching_red_light pass_sign_detected watching_pass_sign')
        self.StepOfParkingLot = Enum('StepOfParkingLot', 'searching_parking_sign searching_parking_point_line searching_nonreserved_parking_area parking')
        self.StepOfLevelCrossing = Enum('StepOfLevelCrossing', 'searching_stop_sign stop_sign_detected searching_level level_detected watching_level stop pass_level')
        self.StepOfTunnel = Enum('StepOfTunnel', 'searching_tunnel_sign tunnel_sign_detected checking_is_in_tunnel in_tunnel_checked searching_light light_detected mazing exit')

        self.current_step_traffic_light = self.StepOfTrafficLight.searching_traffic_light.value
        self.current_step_parking_lot = self.StepOfParkingLot.searching_parking_sign.value
        self.current_step_level_crossing = self.StepOfLevelCrossing.searching_stop_sign.value
        self.current_step_tunnel = self.StepOfTunnel.searching_tunnel_sign.value


        self.Launcher = Enum('Launcher', 'launch_camera launch_detect_sign launch_detect_lane launch_control_lane launch_detect_parking launch_control_parking launch_detect_level launch_control_level launch_detect_tunnel launch_control_tunnel')

#<editor-fold desc="Description">
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        
        self.launch_camera_launched = False
        self.launch_detect_sign_launched = False
        self.launch_detect_lane_launched = False    
        self.launch_control_lane_launched = False
        self.launch_detect_parking_launched = False

        # self.launch_control_parking = roslaunch.scriptapi.ROSLaunch()
        # self.launch_control_parking = roslaunch.parent.ROSLaunchParent(uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_control/launch/turtlebot3_auto_control_parking.launch"])     
        # self.launch_control_parking_launched = False

        # self.launch_detect_level = roslaunch.scriptapi.ROSLaunch()
        # self.launch_detect_level = roslaunch.parent.ROSLaunchParent(uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_detect/launch/turtlebot3_auto_detect_level.launch"])
        # self.launch_detect_level_launched = False

        # self.launch_control_level = roslaunch.scriptapi.ROSLaunch()
        # self.launch_control_level = roslaunch.parent.ROSLaunchParent(uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_control/launch/turtlebot3_auto_control_level.launch"])     
        # self.launch_control_level_launched = False

        # self.launch_detect_tunnel = roslaunch.scriptapi.ROSLaunch()
        # self.launch_detect_tunnel = roslaunch.parent.ROSLaunchParent(uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_detect/launch/turtlebot3_auto_detect_tunnel.launch"])
        # self.launch_detect_tunnel_launched = False

        # self.launch_control_tunnel = roslaunch.scriptapi.ROSLaunch()
        # self.launch_control_tunnel = roslaunch.parent.ROSLaunchParent(uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_control/launch/turtlebot3_auto_control_tunnel.launch"])     
        # self.launch_control_tunnel_launched = False
#</editor-fold>

        self.current_mode = self.CurrentMode.idle.value

        self.is_triggered = False

        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.is_triggered == True:
                self.fnControlNode()
            
            loop_rate.sleep()

    def cbReceiveMode(self, mode_msg):
        rospy.loginfo("starts the progress with %d", mode_msg.data)
        
        self.current_mode = mode_msg.data
        self.is_triggered = True

    # Which step is in Traffic Light
    def cbTrafficLightStamped(self, traffic_light_msg):
        rospy.loginfo("TrafficLight Step changed from %d", self.current_step_traffic_light)

        self.current_step_traffic_light = traffic_light_msg.data

        rospy.loginfo("into %d", self.current_step_traffic_light)

        self.fnControlNode()

        msg_pub_traffic_light_order = UInt8()
        msg_pub_traffic_light_order.data = self.current_step_traffic_light
        self.pub_traffic_light_order.publish(msg_pub_traffic_light_order)

    # Which step is in Parking Lot
    def cbParkingLotStamped(self, parking_lot_msg):
        rospy.loginfo("ParkingLot Step changed from %d", self.current_step_parking_lot)

        self.current_step_parking_lot = parking_lot_msg.data

        rospy.loginfo("into %d", self.current_step_parking_lot)

        rospy.sleep(2)

        if self.current_step_parking_lot == self.StepOfParkingLot.searching_parking_sign.value:
            self.current_mode = self.CurrentMode.lane_following.value
            msg_mode_return = UInt8()
            msg_mode_return.data = self.current_mode
            self.pub_mode_return.publish(msg_mode_return)

        self.is_triggered = True

    # Which step is in Level Crossing  
    def cbLevelCrossingStamped(self, level_crossing_msg):
        self.fnDecide(self.CallNumber.level_crossing.value, level_crossing_msg.data)

    # Which step is in Tunnel
    def cbTunnelStamped(self, tunnel_msg):
        self.fnDecide(self.CallNumber.tunnel.value, tunnel_msg.data)


    def fnControlNode(self):
        if self.current_mode == self.CurrentMode.lane_following.value:
            rospy.loginfo("New trigger for lane_following")

            self.fnLaunch(self.Launcher.launch_camera.value, True)
            self.fnLaunch(self.Launcher.launch_detect_sign.value, True)
            self.fnLaunch(self.Launcher.launch_detect_lane.value, True)
            self.fnLaunch(self.Launcher.launch_control_lane.value, True)
            self.fnLaunch(self.Launcher.launch_detect_parking.value, False)
            # self.fnLaunch(self.Launcher.launch_control_parking.value, False)
            # self.fnLaunch(self.Launcher.launch_detect_level.value, False)
            # self.fnLaunch(self.Launcher.launch_control_level.value, False)
            # self.fnLaunch(self.Launcher.launch_detect_tunnel.value, False)
            # self.fnLaunch(self.Launcher.launch_control_tunnel.value, False)
              

        # parking_lot
        elif self.current_mode == self.CurrentMode.parking_lot.value:
            rospy.loginfo("New trigger for parking_lot")
            msg_pub_parking_lot_order = UInt8()

            if self.current_step_parking_lot == self.StepOfParkingLot.searching_parking_sign.value:
                rospy.loginfo("Current step : searching_parking_sign")
                rospy.loginfo("Go to next step : searching_parking_point_line")

                msg_pub_parking_lot_order.data = self.StepOfParkingLot.searching_parking_point_line.value
                               
                self.fnLaunch(self.Launcher.launch_camera.value, True)
                self.fnLaunch(self.Launcher.launch_detect_sign.value, False)
                self.fnLaunch(self.Launcher.launch_detect_lane.value, True)
                self.fnLaunch(self.Launcher.launch_control_lane.value, True)
                self.fnLaunch(self.Launcher.launch_detect_parking.value, True)
                # self.fnLaunch(self.Launcher.launch_control_parking.value, False)
                # self.fnLaunch(self.Launcher.launch_detect_level.value, False)
                # self.fnLaunch(self.Launcher.launch_control_level.value, False)
                # self.fnLaunch(self.Launcher.launch_detect_tunnel.value, False)
                # self.fnLaunch(self.Launcher.launch_control_tunnel.value, False)
                                   
            elif self.current_step_parking_lot == self.StepOfParkingLot.searching_parking_point_line.value:
                rospy.loginfo("Current step : searching_parking_point_line")
                rospy.loginfo("Go to next step : searching_nonreserved_parking_area")

                msg_pub_parking_lot_order.data = self.StepOfParkingLot.searching_nonreserved_parking_area.value

                self.fnLaunch(self.Launcher.launch_camera.value, True)
                self.fnLaunch(self.Launcher.launch_detect_sign.value, False)
                self.fnLaunch(self.Launcher.launch_detect_lane.value, False)
                self.fnLaunch(self.Launcher.launch_control_lane.value, False)
                self.fnLaunch(self.Launcher.launch_detect_parking.value, True)
                # self.fnLaunch(self.Launcher.launch_control_parking.value, False)
                # self.fnLaunch(self.Launcher.launch_detect_level.value, False)
                # self.fnLaunch(self.Launcher.launch_control_level.value, False)
                # self.fnLaunch(self.Launcher.launch_detect_tunnel.value, False)
                # self.fnLaunch(self.Launcher.launch_control_tunnel.value, False)

            elif self.current_step_parking_lot == self.StepOfParkingLot.searching_nonreserved_parking_area.value:
                rospy.loginfo("Current step : searching_nonreserved_parking_area")
                rospy.loginfo("Go to next step : parking")

                msg_pub_parking_lot_order.data = self.StepOfParkingLot.parking.value

                self.fnLaunch(self.Launcher.launch_camera.value, True)
                self.fnLaunch(self.Launcher.launch_detect_sign.value, False)
                self.fnLaunch(self.Launcher.launch_detect_lane.value, False)
                self.fnLaunch(self.Launcher.launch_control_lane.value, False)
                self.fnLaunch(self.Launcher.launch_detect_parking.value, True)
                # self.fnLaunch(self.Launcher.launch_control_parking.value, False)
                # self.fnLaunch(self.Launcher.launch_detect_level.value, False)
                # self.fnLaunch(self.Launcher.launch_control_level.value, False)
                # self.fnLaunch(self.Launcher.launch_detect_tunnel.value, False)
                # self.fnLaunch(self.Launcher.launch_control_tunnel.value, False)

            elif self.current_step_parking_lot == self.StepOfParkingLot.parking.value:
                rospy.loginfo("Current step : parking")
                rospy.loginfo("Go to next step : searching_parking_sign")

                msg_pub_parking_lot_order.data = self.StepOfParkingLot.searching_parking_sign.value

                # self.current_mode = self.CurrentMode.lane_following.value

                self.fnLaunch(self.Launcher.launch_camera.value, True)
                self.fnLaunch(self.Launcher.launch_detect_sign.value, True)
                self.fnLaunch(self.Launcher.launch_detect_lane.value, True)
                self.fnLaunch(self.Launcher.launch_control_lane.value, True)
                self.fnLaunch(self.Launcher.launch_detect_parking.value, True)
                # self.fnLaunch(self.Launcher.launch_control_parking.value, False)
                # self.fnLaunch(self.Launcher.launch_detect_level.value, False)
                # self.fnLaunch(self.Launcher.launch_control_level.value, False)
                # self.fnLaunch(self.Launcher.launch_detect_tunnel.value, False)
                # self.fnLaunch(self.Launcher.launch_control_tunnel.value, False)

            rospy.sleep(2)

            self.pub_parking_lot_order.publish(msg_pub_parking_lot_order)

        # elif self.mode_current == self.Mode.level_crossing.value:
        #     if self.mode_previous == self.Mode.lane_following.value:
        #         self.launch_detect_lane.shutdown()
        #         self.launch_control_lane.shutdown()
        #     elif self.mode_previous == self.Mode.parking.value:
        #         pass
        #     elif self.mode_previous == self.Mode.tunnel.value:
        #         pass
        
        # elif self.mode_current == self.Mode.tunnel.value:
        #     if self.mode_previous == self.Mode.lane_following.value:
        #         self.launch_detect_lane.shutdown()
        #         self.launch_control_lane.shutdown()
        #     elif self.mode_previous == self.Mode.parking.value:
        #         pass
        #     elif self.mode_previous == self.Mode.level_crossing.value:
        #         pass
        # else:
        #     pass

        self.is_triggered = False

    def fnLaunch(self, launch_num, is_start):
        if launch_num == self.Launcher.launch_camera.value:
            if is_start == True:
                if self.launch_camera_launched == False:
                    self.launch_camera = roslaunch.scriptapi.ROSLaunch()
                    self.launch_camera = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_camera/launch/turtlebot3_auto_camera_calibration.launch"])
                    self.launch_camera_launched = True
                    self.launch_camera.start()
                else:
                    pass
            else:
                if self.launch_camera_launched == True:
                    self.launch_camera_launched = False
                    self.launch_camera.shutdown()
                else:
                    pass
        elif launch_num == self.Launcher.launch_detect_sign.value:
            if is_start == True:
                if self.launch_detect_sign_launched == False:
                    self.launch_detect_sign = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_sign = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_detect/launch/turtlebot3_auto_detect_sign.launch"])
                    self.launch_detect_sign_launched = True
                    self.launch_detect_sign.start()
                else:
                    pass
            else:
                if self.launch_detect_sign_launched == True:
                    self.launch_detect_sign_launched = False
                    self.launch_detect_sign.shutdown()
                else:
                    pass                
        elif launch_num == self.Launcher.launch_detect_lane.value:
            if is_start == True:
                if self.launch_detect_lane_launched == False:
                    self.launch_detect_lane = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_lane = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_detect/launch/turtlebot3_auto_detect_lane.launch"])
                    self.launch_detect_lane_launched = True
                    self.launch_detect_lane.start()
                else:
                    pass
            else:
                if self.launch_detect_lane_launched == True:
                    self.launch_detect_lane_launched = False
                    self.launch_detect_lane.shutdown()
                else:
                    pass                  
        elif launch_num == self.Launcher.launch_control_lane.value:
            if is_start == True:
                if self.launch_control_lane_launched == False:
                    self.launch_control_lane = roslaunch.scriptapi.ROSLaunch()
                    self.launch_control_lane = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_control/launch/turtlebot3_auto_control_lane.launch"])
                    self.launch_control_lane_launched = True
                    self.launch_control_lane.start()
                else:
                    pass
            else:
                if self.launch_control_lane_launched == True:
                    self.launch_control_lane_launched = False
                    self.launch_control_lane.shutdown()
                else:
                    pass                  
        elif launch_num == self.Launcher.launch_detect_parking.value:
            if is_start == True:
                if self.launch_detect_parking_launched == False:
                    self.launch_detect_parking = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_parking = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_detect/launch/turtlebot3_auto_detect_parking.launch"])
                    self.launch_detect_parking_launched = True
                    self.launch_detect_parking.start()
                else:
                    pass
            else:
                if self.launch_detect_parking_launched == True:
                    self.launch_detect_parking_launched = False
                    self.launch_detect_parking.shutdown()
                else:
                    pass                  
        # elif launch_num == self.Launcher.launch_control_parking.value:
        #     if is_start == True:
        #         if self.launch_control_parking_launched == False:
        #             self.launch_control_parking_launched = True
        #             self.launch_control_parking.start()
        #         else:
        #             pass
        #     else:
        #         if self.launch_control_parking_launched == True:
        #             self.launch_control_parking_launched = False
        #             self.launch_control_parking.shutdown()
        #         else:
        #             pass                  
        # elif launch_num == self.Launcher.launch_detect_level.value:
        #     if is_start == True:
        #         if self.launch_detect_level_launched == False:
        #             self.launch_detect_level_launched = True
        #             self.launch_detect_level.start()
        #         else:
        #             pass
        #     else:
        #         if self.launch_detect_level_launched == True:
        #             self.launch_detect_level_launched = False
        #             self.launch_detect_level.shutdown()
        #         else:
        #             pass                  
        # elif launch_num == self.Launcher.launch_control_level.value:
        #     if is_start == True:
        #         if self.launch_control_level_launched == False:
        #             self.launch_control_level_launched = True
        #             self.launch_control_level.start()
        #         else:
        #             pass
        #     else:
        #         if self.launch_control_level_launched == True:
        #             self.launch_control_level_launched = False
        #             self.launch_control_level.shutdown()
        #         else:
        #             pass                  
        # elif launch_num == self.Launcher.launch_detect_tunnel.value:
        #     if is_start == True:
        #         if self.launch_detect_tunnel_launched == False:
        #             self.launch_detect_tunnel_launched = True
        #             self.launch_detect_tunnel.start()
        #         else:
        #             pass
        #     else:
        #         if self.launch_detect_tunnel_launched == True:
        #             self.launch_detect_tunnel_launched = False
        #             self.launch_detect_tunnel.shutdown()
        #         else:
        #             pass                  
        # elif launch_num == self.Launcher.launch_control_tunnel.value:
        #     if is_start == True:
        #         if self.launch_control_tunnel_launched == False:
        #             self.launch_control_tunnel_launched = True
        #             self.launch_control_tunnel.start()
        #         else:   
        #             pass
        #     else:
        #         if self.launch_control_tunnel_launched == True:
        #             self.launch_control_tunnel_launched = False
        #             self.launch_control_tunnel.shutdown()
        #         else:
        #             pass                  

    def main(self):
        rospy.spin()


        # if call_number == self.CallNumber.traffic_sign.value:
        #     # parking_lot
        #     if msg_num == 0:
        #         self.current_state_parking_lot = self.StateParkingLot.searching_parking_sign.value
        #     elif msg_num == 1:
        #         self.current_state_parking_lot = self.StateParkingLot.parking_sign_detected.value
        #     elif msg_num == 2:
        #         self.current_state_parking_lot = self.StateParkingLot.searching_parking_point_line.value
        #     elif msg_num == 3:
        #         self.current_state_parking_lot = self.StateParkingLot.parking_point_line_detected.value
        #     elif msg_num == 4:
        #         self.current_state_parking_lot = self.StateParkingLot.searching_nonreserved_parking_area.value        
        #     elif msg_num == 5:
        #         self.current_state_parking_lot = self.StateParkingLot.nonreserved_parking_area_detected.value
        #     elif msg_num == 6:
        #         self.current_state_parking_lot = self.StateParkingLot.parking.value
        #     elif msg_num == 7:
        #         self.current_state_parking_lot = self.StateParkingLot.exit.value

        #     # level_crossing
        #     elif msg_num == 8:
        #         self.current_state_level_crossing = self.StateLevelCrossing.searching_stop_sign.value
        #     elif msg_num == 9:
        #         self.current_state_level_crossing = self.StateLevelCrossing.stop_sign_detected.value
        #     elif msg_num == 10:
        #         self.current_state_level_crossing = self.StateLevelCrossing.searching_level.value
        #     elif msg_num == 11:
        #         self.current_state_level_crossing = self.StateLevelCrossing.level_detected.value        
        #     elif msg_num == 12:
        #         self.current_state_level_crossing = self.StateLevelCrossing.watching_level.value
        #     elif msg_num == 13:
        #         self.current_state_level_crossing = self.StateLevelCrossing.stop.value
        #     elif msg_num == 14:
        #         self.current_state_level_crossing = self.StateLevelCrossing.pass_level.value

        #     # tunnel
        #     elif msg_num == 15:
        #         self.current_state_tunnel = self.StateTunnel.searching_tunnel_sign.value
        #     elif msg_num == 16:
        #         self.current_state_tunnel = self.StateTunnel.tunnel_sign_detected.value
        #     elif msg_num == 17:
        #         self.current_state_tunnel = self.StateTunnel.checking_is_in_tunnel.value
        #     elif msg_num == 18:
        #         self.current_state_tunnel = self.StateTunnel.in_tunnel_checked.value        
        #     elif msg_num == 19:
        #         self.current_state_tunnel = self.StateTunnel.searching_light.value
        #     elif msg_num == 20:
        #         self.current_state_tunnel = self.StateTunnel.light_detected.value
        #     elif msg_num == 21:
        #         self.current_state_tunnel = self.StateTunnel.mazing.value
        #     elif msg_num == 22:
        #         self.current_state_tunnel = self.StateTunnel.exit.value

        # if call_number == self.CallNumber.traffic_light.value:



if __name__ == '__main__':
    rospy.init_node('core_node_controller')
    node = CoreNodeController()
    node.main()
