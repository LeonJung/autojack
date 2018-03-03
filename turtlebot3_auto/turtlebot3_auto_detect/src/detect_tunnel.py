#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8, Float64, String
from enum import Enum
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
import math
import tf

class DetectTunnel():
    def __init__(self):
        # subscribes state : white line reliability
        # self.sub_white_line_reliability = rospy.Subscriber('/detect/white_line_reliability', UInt8, self.cbWhiteLineReliable, queue_size=1)
 
        self.sub_tunnel_order = rospy.Subscriber('/detect/tunnel_order', UInt8, self.cbTunnelOrder, queue_size=1)

        self.sub_arrival_status = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.cbGetNavigationResult, queue_size=1)

        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        # self.sub_obstacle = rospy.Subscriber('/detect/obstacle', UInt8, self.cbObstacleDetected, queue_size=1)

        # self.sub_level_crossing_finished = rospy.Subscriber('/control/level_crossing_finished', UInt8, self.cbLevelCrossingFinished, queue_size = 1)

        self.pub_tunnel_return = rospy.Publisher('/detect/tunnel_stamped', UInt8, queue_size=1)

        # self.pub_parking_start = rospy.Publisher('/control/parking_start', UInt8, queue_size = 1)

        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)

        self.pub_goal_pose_stamped = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)

        self.StepOfTunnel = Enum('StepOfTunnel', 'searching_tunnel_sign go_in_to_tunnel navigation go_out_from_tunnel exit')

        # self.position_turning_point = None
        # self.theta_turning_point = None

        # self.position_parking = None

        self.is_navigation_finished = False

        self.is_tunnel_finished = False

        self.last_current_theta = 0.0

        # rospy.sleep(1)


        # self.fnPubGoalPose()

        # loop_rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.fnPubGoalPose()

        #     loop_rate.sleep()

    def cbGetNavigationResult(self, msg_nav_result):
        if msg_nav_result.status.status == 3:
            rospy.loginfo("Reached")
            self.is_navigation_finished = True


    def cbTunnelOrder(self, order):
        pub_tunnel_return = UInt8()

        if order.data == self.StepOfTunnel.searching_tunnel_sign.value:
            rospy.loginfo("Now lane_following")

            pub_tunnel_return.data = self.StepOfTunnel.searching_tunnel_sign.value
                            
                                
        elif order.data == self.StepOfTunnel.go_in_to_tunnel.value:
            rospy.loginfo("Now go_in_to_tunnel")

            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y

            while True:
                error = self.fnStraight(0.35)

                if math.fabs(error) < 0.005:
                    break

            self.fnStop()

            rospy.loginfo("go_in_to_tunnel finished")

            pub_tunnel_return.data = self.StepOfTunnel.go_in_to_tunnel.value


        elif order.data == self.StepOfTunnel.navigation.value:
            rospy.loginfo("Now navigation")

            self.fnPubGoalPose()

            while True:
                if self.is_navigation_finished == True:
                    break
                else:
                    pass

            pub_tunnel_return.data = self.StepOfTunnel.navigation.value


        elif order.data == self.StepOfTunnel.go_out_from_tunnel.value:
            rospy.loginfo("Now go_out_from_tunnel")

            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y

            while True:
                error = self.fnStraight(0.25)

                if math.fabs(error) < 0.005:
                    break

            self.fnStop()

            pub_tunnel_return.data = self.StepOfTunnel.go_out_from_tunnel.value

        elif order.data == self.StepOfTunnel.exit.value:
            rospy.loginfo("Now exit")

            # while True:
            #     if self.is_obstacle_detected == False:
            #         rospy.loginfo("parking lot is clear")
            #         break
            #     else:
            #         rospy.loginfo("right side is full")

            pub_tunnel_return.data = self.StepOfTunnel.exit.value

        self.pub_tunnel_return.publish(pub_tunnel_return)

    def cbOdom(self, odom_msg):
		#  (self.now_roll, self.now_pitch, self.now_yaw) = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)

        # rospy.loginfo("Got current theta : %f", self.current_theta)

        if (self.current_theta - self.last_current_theta) < -math.pi:
            # rospy.loginfo("subtract current theta : %f", self.current_theta - self.last_current_theta)
            # rospy.loginfo("it is gone to minus")
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            # rospy.loginfo("subtract current theta : %f", self.current_theta - self.last_current_theta)
            # rospy.loginfo("it is gone to plus")
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        # rospy.loginfo("mod current theta : %f", self.current_theta)
        # rospy.loginfo("last current theta : %f", self.last_current_theta)

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y
        # rospy.loginfo(self.)

    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        # if theta < 0:
        #     theta = theta + np.pi * 2
        # if theta > np.pi * 2:
        #     theta = theta - np.pi * 2
        return theta


    def fnPubGoalPose(self):
        goalPoseStamped = PoseStamped()

        goalPoseStamped.header.frame_id = "map"
        goalPoseStamped.header.stamp = rospy.Time.now()

        goalPoseStamped.pose.position.x = 1.50108861923#1.49971497059 
        goalPoseStamped.pose.position.y = 1.36013686657#1.46927690506
        goalPoseStamped.pose.position.z = 0.0

        goalPoseStamped.pose.orientation.x = 0.0
        goalPoseStamped.pose.orientation.y = 0.0
        goalPoseStamped.pose.orientation.z = 0.707669819124#0.701553189571
        goalPoseStamped.pose.orientation.w = 0.70654329457#0.712617093679

        self.pub_goal_pose_stamped.publish(goalPoseStamped)

    def fnStraight(self, desired_dist):
        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
        
        rospy.loginfo("Tunnel_Straight")
        rospy.loginfo("err_pos  desired_dist : %f  %f  %f", err_pos, desired_dist, self.lastError)

        Kp = 0.4#0.15
        Kd = 0.05#0.07

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()
        twist.linear.x = 0.07
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        # rospy.loginfo("angular_z : %f", angular_z)

        return err_pos

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def cbTunnelFinished(self, tunnel_finished_msg):
        self.is_tunnel_finished = True

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_tunnel')
    node = DetectTunnel()
    node.main()
