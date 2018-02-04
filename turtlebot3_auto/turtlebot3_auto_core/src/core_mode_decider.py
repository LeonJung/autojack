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
import numpy as np
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image, CompressedImage

from enum import Enum

class CoreModeDecider():
    def __init__(self):
        # subscribes state : traffic sign detected
        self.sub_traffic_sign = rospy.Subscriber('/detect/traffic_sign', UInt8, self.cbInvokedByTrafficSign, queue_size=1)

        # subscribes state : traffic light detected
        self.sub_traffic_light = rospy.Subscriber('/detect/traffic_light', UInt8, self.cbInvokedByTrafficLight, queue_size=1)
        
        # subscribes state : parking_lot detected
        self.sub_parking_lot = rospy.Subscriber('/detect/parking_lot', UInt8, self.cbChangeStatusParkingLot, queue_size=1)

        # subscribes state : level crossing detected
        self.sub_level_crossing = rospy.Subscriber('/detect/level_crossing', UInt8, self.cbChangeStatusLevelCrossing, queue_size=1)

        # subscribes state : tunnel detected
        self.sub_tunnel = rospy.Subscriber('/detect/tunnel', UInt8, self.cbChangeStatusTunnel, queue_size=1)   

        # self._sub_2 = rospy.Subscriber('/stop_bar', Stop_bar, self.receiver_stop_bar, queue_size=1)
        # self._sub_4 = rospy.Subscriber('/parking', String, self.receiver_parking, queue_size=1)
        # self._sub_5 = rospy.Subscriber('/scan', LaserScan, self.callback2, queue_size=1)
        # self._sub_6 = rospy.Subscriber('/maze', String, self.receiver_maze, queue_size=1)
        # self._sub_7 = rospy.Subscriber('/signal_sign', String, self.receiver_signal_sign, queue_size=1)
        # self._sub_8 = rospy.Subscriber('/objects', Float32MultiArray, self.receiver_object_find, queue_size=1)
        # self._pub_1 = rospy.Publisher('/command_lane_follower', String, queue_size=1)
        # self._pub_2 = rospy.Publisher('/command_maze', String, queue_size=1)

        self.TrafficSign = Enum('TrafficSign', 'stop divide parking tunnel')

        self.CurrentMode = Enum('CurrentMode', 'idle traffic_light parking_lot level_crossing tunnel')
        self.current_mode = self.CurrentMode.idle.value

        self.InvokedObject = Enum('InvokedObject', 'traffic_sign traffic_light')

        # self.Trea

        # self.Mode

        # self.StateTrafficLight = Enum('StateTrafficLight', 'searching_traffic_light traffic_light_detected watching_green_light yellow_light_detected watching yellow_light red_light_detected watching_red_light pass_sign_detected watching_pass_sign')
        # self.StateParkingLot = Enum('StateParkingLot', 'searching_parking_sign parking_sign_detected searching_parking_point_line parking_point_line_detected searching_nonreserved_parking_area nonreserved_parking_area_detected parking exit')
        # self.StateLevelCrossing = Enum('StateLevelCrossing', 'searching_stop_sign stop_sign_detected searching_level level_detected watching_level stop pass_level')
        # self.StateTunnel = Enum('StateTunnel', 'searching_tunnel_sign tunnel_sign_detected checking_is_in_tunnel in_tunnel_checked searching_light light_detected mazing exit')

        # self.current_state_traffic_light = self.StateTrafficLight.searching_traffic_light.value
        # self.current_state_parking_lot = self.StateParkingLot.searching_parking_sign.value
        # self.current_state_level_crossing = self.StateLevelCrossing.searching_stop_sign.value
        # self.current_state_tunnel = self.StateTunnel.searching_tunnel_sign.value

    # Invoke if traffic sign is detected
    def cbInvokedByTrafficSign(self, traffic_sign_type_msg):
        self.fnDecide(self.InvokedObject.traffic_sign.value, traffic_sign_type_msg.data)

    # Invoke if traffic light is detected
    def cbInvokedByTrafficLight(self, traffic_light_type_msg):
        self.fnDecide(self.InvokedObject.traffic_light.value, traffic_light_type_msg.data)

    # # Which step is in Parking Lot
    # def cbChangeStatuspParkingLot(self, parking_lot_msg):
    #     self.fnDecide(self.CallNumber.parking_lot.value, parking_lot_msg.data)

    # # Which step is in Level Crossing  
    # def cbChangeStatusLevelCrossing(self, level_status_msg):
    #     self.fnDecide(self.CallNumber.level_crossing.value, level_status_msg.data)

    # # Which step is in tunnel
    # def cbChangeStatusTunnel(self, tunnel_msg):
    #     self.fnDecide(self.CallNumber.tunnel.value, tunnel_msg.data)

    def fnDecide(self, invoked_object, msg_data):
        if self.current_mode == self.CurrentMode.idle.value:
            if invoked_object == self.InvokedObject.traffic_light.value:    # Traffic Light detected
                self.current_mode = self.CurrentMode.traffic_light.value
            elif invoked_object == self.InvokedObject.traffic_sign.value:
                if msg_data.data == self.TrafficSign.stop.value:            # Stop Sign detected
                    self.current_mode = self.CurrentMode.level_crossing.value
                elif msg_data.data == self.TrafficSign.divide.value:        # Divide Sign detected
                    pass
                elif msg_data.data == self.TrafficSign.parking.value:       # Parking Sign detected
                    self.current_mode = self.CurrentMode.parking_lot.value
                elif msg_data.data == self.TrafficSign.tunnel.value:        # Tunnel Sign detected
                    self.current_mode = self.CurrentMode.tunnel.value
            else:
                pass
        else:
            pass
        


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
    rospy.init_node('core_mode_decider')
    node = CoreModeDecider()
    node.main()
