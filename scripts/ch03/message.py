#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32


def callback(msg):
    doubled = Int32()
    doubled.data = msg.data * 2
    print doubled.data
    pub.publish(doubled)


rospy.init_node('doubler')

pub = rospy.Publisher('doubled', Int32)
pub.publish(Int32(1))
sub = rospy.Subscriber('doubled', Int32, callback)

rospy.spin()
