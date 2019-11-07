#! /usr/bin/env python


import math


import rospy
from sensor_msgs.msg import LaserScan


def scan_callback(msg):
    range_ahead = msg.ranges[len(msg.ranges) / 2]
    angle_min_in_degree = msg.angle_min * 180.0 / math.pi
    angle_max_in_degree = msg.angle_max * 180.0 / math.pi
    angle_inc_in_degree = msg.angle_increment * 180.0 / math.pi
    rospy.loginfo('angle_min = %f, angle_max = %f, angle_inc = %f', angle_min_in_degree, angle_max_in_degree, angle_inc_in_degree)
    print "range ahead: %0.1f" % range_ahead


rospy.init_node('range_ahead')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()