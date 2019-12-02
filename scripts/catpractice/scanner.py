#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Laser_scanner:
    def __init__(self):
        self.range_ahead = 0.0
        self.range_left = 0.0
        self.range_right = 0.0
        self.left = 0.0
        self.right = 0.0
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        #rospy.spin()

    def scan_callback(self, msg):
        #get right angle ranges
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]  # 180
        self.right = msg.ranges[len(msg.ranges) / 4]      # 90
        self.left = msg.ranges[len(msg.ranges) * 3 / 4]  # 270

        #self.max_range_point = round(list.index(max(msg.ranges), msg.ranges) / len(msg.ranges) * 360.0, 2)

        #get nearest range
        #2.0 > right or left
        self.range_right_min = reduce(lambda x, y: x if x < y else y, msg.ranges[len(msg.ranges) / 4:len(msg.ranges) * 3 / 8])
        self.range_left_min = reduce(lambda x, y: x if x < y else y, msg.ranges[len(msg.ranges) * 5 / 8:len(msg.ranges) * 3 / 4])
        self.range_right_max = reduce(lambda x, y: x if x > y else y, msg.ranges[len(msg.ranges) / 8:len(msg.ranges) * 3 / 8])
        self.range_left_max = reduce(lambda x, y: x if x > y else y, msg.ranges[len(msg.ranges) * 5 / 8:len(msg.ranges) * 7 / 8])
        #self.print_laser_status()

    def print_laser_status(self):
        print "range ahead: %0.1f" % self.range_ahead
        print "range left min: %0.1f" % self.range_left_min
        print "range right min: %0.1f" % self.range_right_min
        print "range left max: %0.1f" % self.range_left_max
        print "range right max: %0.1f" % self.range_right_max
        print "left: %0.1f" % self.left
        print "right: %0.1f" % self.right


class Pose_scanner:
    def __init__(self):
        self.angle = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.position = [0, 0]
        self.pose_sub = rospy.Subscriber('odom', Odometry, self.pose_callback)

    def pose_callback(self, msg):
        ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler = euler_from_quaternion(ori)
        self.angle = round((euler[2] * 180.0 / math.pi) + 180.0, 2) #use yaw, +-15.0
        '''
        self.position_x = round(msg.pose.pose.position.x, 2)
        self.position_y = round(msg.pose.pose.position.y, 2)
        self.position = [self.position_x, self.position_y]
        '''