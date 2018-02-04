 #!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import tf

def callback(x):
    pass

def sign(num):
    if num > 0:
        return 1
    else:
        return -1

def theta_dot2dot(start, end):
    theta = math.atan2(end[1]-start[1], end[0]-start[0])
    return theta

def distance_dot2line(a, b, c, x0, y0):
    distance = abs(x0*a + y0*b + c)/math.sqrt(a*a + b*b)
    sign_ = sign(x0*a + y0*b + c) * sign(a)
    return sign_, distance

def distance_dot2dot(point1, point2):
    return math.sqrt((point2[1] - point1[1]) * (point2[1] - point1[1]) + (point2[0] - point1[0]) * (point2[0] - point1[0]))

def position_y(arr):
    return arr[1]

def sotring_dotted_line(arr):
    arr.sort(key = position_y, reverse=True)
    done = 0
    while(not done):
        for i in range(len(arr)):
            if i == len(arr)-1:
                done = 1
                break
            if distance_dot2dot(arr[i], arr[i+1]) < 10:
                arr[i:i+2] = [arr[i]]
                break
    return arr

def euler_from_quaternion(quaternion):
    theta = tf.transformations.euler_from_quaternion(quaternion)[2]
    if theta < 0:
        theta = theta + np.pi * 2
    if theta > np.pi * 2:
        theta = theta - np.pi * 2
    return theta

def setting_parking_position(position_turning_point, theta_turning_point, distance):
    parking_point_x = position_turning_point[0] + math.cos(theta_turning_point - np.pi/2) * distance
    parking_point_y = position_turning_point[1] + math.sin(theta_turning_point - np.pi/2) * distance
    return [parking_point_x, parking_point_y]





class DetectObstacle():
    def __init__(self):
        # subscribes state : yellow line reliability
        self.sub_traffic_sign = rospy.Subscriber('/detect/yellow_line_reliability', UInt8, self.cbChangeStatusTrafficSign, queue_size=1)
 
        # subscribes state : white line reliability
        self.sub_traffic_sign = rospy.Subscriber('/detect/white_line_reliability', UInt8, self.cbChangeStatusTrafficSign, queue_size=1)
 

        self._sub_2 = rospy.Subscriber('/detect/obstacle', UInt8, self.callback2, queue_size=1)
        # self._sub_3 = rospy.Subscriber('/odom', Odometry, self.callback3, queue_size=1)

        self._pub1 = rospy.Publisher('/detect/parking_lot', , queue_size=1)
        # self._pub2 = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        # Two variables below will be used when there is no lane
        # self.lane_position : -1 means lane is in left and 1 is opposite
        # When there is no lane detected, turtlebot will turn to the direction where lane was existed in the last cycle
        self.lane_position = 0

        self.state = 'checking_dotted_line'
        self.parking_lot_num = 1

        self.position_now = None
        self.theta_now = None

        self.position_parking_detected = None

        self.position_turning_point = None
        self.theta_turning_point = None

        self.position_parking = None

    def callback2(self, scan):
        # angle_scan = 15
        # scan_start = 270 - angle_scan
        # scan_end = 270 + angle_scan
        # threshold_distance = 0.3
        # obstacle_existence = 'no'
        # obstacle_count = 0



        angle_scan = 45
        scan_start = 270 - angle_scan
        scan_end = 270 + angle_scan
        threshold_distance = 0.5
        obstacle_existence = False
        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                obstacle_existence = True




        print obstacle_existence





        # if self.state == 'checking_obstacle':
        #     angle_scan = 15
        #     scan_start = 270 - angle_scan
        #     scan_end = 270 + angle_scan
        #     threshold_distance = 0.5
        #     obstacle_existence = 'no'
        #     for i in range(scan_start, scan_end):
        #         if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
        #             obstacle_existence = 'yes'
        #             print 'obstacle in', i
        #     if obstacle_existence == 'no':
        #         message = "parking_lot_detected"
        #         self._pub1.publish(message)
        #         self.state = 'parking'
        #     elif obstacle_existence == 'yes':
        #         if self.parking_lot_num == 1:
        #             self.parking_lot_num = 2
        #             self.state = 'setting_parking_position'
        #         elif self.parking_lot_num == 2:
        #             self.state = 'parking_end'




    # def callback3(self, odometry):
    #     self.position_now = [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
    #     quaternion = (odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w)
    #     self.theta_now = euler_from_quaternion(quaternion)
    #     #print self.theta_now
    #     #print self.position_now
    #     if self.state == 'setting_parking_position':
    #         if self.position_parking_detected == None:
    #             self.position_parking_detected = self.position_now
    #         else:
    #             if self.parking_lot_num == 1:
    #                 print distance_dot2dot(self.position_parking_detected, self.position_now)
    #                 if distance_dot2dot(self.position_parking_detected, self.position_now) > 0.3:
    #                     self.state = 'checking_obstacle'
    #             elif self.parking_lot_num == 2:
    #                 if distance_dot2dot(self.position_parking_detected, self.position_now) > 0.85:
    #                     self.state = 'checking_obstacle'

    #     elif self.state == 'parking':
    #         print self.position_parking
    #         if self.position_turning_point == None and self.theta_turning_point == None:
    #             self.position_turning_point = self.position_now
    #             self.theta_turning_point = self.theta_now
    #             parking_lot_distance = 0.25
    #             self.position_parking = setting_parking_position(self.position_turning_point, self.theta_turning_point, parking_lot_distance)
    #         else:
    #             distance_from_parking_position = distance_dot2dot(self.position_now, self.position_parking)
    #             if distance_from_parking_position < 0.02:
    #                 self.state = 'return_to_lane'
    #             else:
    #                 self.move_to_some_point(self.position_now, self.theta_now, self.position_parking)


    #     elif self.state == 'return_to_lane':
    #         distance_from_turning_position = distance_dot2dot(self.position_now, self.position_turning_point)
    #         if distance_from_turning_position < 0.02:
    #             self.state = 'align to lane'
    #         else:
    #             self.move_to_some_point(self.position_now, self.theta_now, self.position_turning_point)

    #     elif self.state == 'align to lane':
    #         if abs(self.theta_now - self.theta_turning_point) < np.pi/100:
    #             self.state = 'parking_end'
    #             message = "parking_lot_end"
    #             self._pub1.publish(message)
    #         else:
    #             self.setting_angle(self.theta_now, self.theta_turning_point)





    # def move_to_some_point(self, position_now, theta_now, position_desired):
    #     theta_desired = theta_dot2dot(position_now, position_desired)
    #     diff = abs(theta_desired - theta_now)
    #     if diff > 2*np.pi:
    #         diff -= 2*np.pi
    #     if diff > np.pi/100:
    #         print 'diff', abs(theta_desired - theta_now)
    #         self.setting_angle(theta_now, theta_desired)
    #     else:
    #         self.going_straight()



    # def setting_angle(self, theta_now, theta_desired):
    #     if theta_desired < 0:
    #         theta_desired += np.pi*2
    #     print 'setting angle'
    #     print theta_now
    #     print theta_desired
    #     if theta_desired > theta_now:
    #         if theta_desired - theta_now < np.pi:
    #             turn_direction = 'left'
    #         else:
    #             turn_direction = 'right'
    #     else:
    #         if theta_now - theta_desired < np.pi:
    #             turn_direction = 'right'
    #         else:
    #             turn_direction = 'left'
    #             # publish topic
    #     difference = abs(theta_desired - theta_now)
    #     if difference > np.pi:
    #         difference = np.pi * 2 - difference
    #     if difference > 0.3:
    #         turn_speed = 0.6
    #     elif difference > 0.2:
    #         turn_speed = 0.3
    #     elif difference > 0.1:
    #         turn_speed = 0.1
    #     elif difference > 0.01:
    #         turn_speed = 0.05
    #     else:
    #         turn_speed = 0
    #     if turn_direction == 'left':
    #         ang_z = turn_speed
    #     else:
    #         ang_z = - turn_speed
    #     self.publishing_vel(0, 0, 0, 0, 0, ang_z)

    # def going_straight(self):
    #     print 'going straight'
    #     print self.position_now
    #     print self.theta_now
    #     print self.position_parking
    #     self.publishing_vel(0.06, 0, 0, 0, 0, 0)


    # def publishing_vel(self, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z):
    #     vel = Twist()
    #     vel.angular.x = ang_x
    #     vel.angular.y = ang_y
    #     vel.angular.z = ang_z
    #     vel.linear.x = lin_x
    #     vel.linear.y = lin_y
    #     vel.linear.z = lin_z
    #     self._pub2.publish(vel)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_obstacle')
    node = DetectObstacle()
    node.main()
