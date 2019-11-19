#! /usr/bin/env python

'''
Controller side
'''

import rospy
from Keys import Keys
import sys, select, tty, termios

if __name__ == "__main__":
    rospy.init_node('drive_control')
    rate = rospy.Rate(10)

    keys = Keys()
    while not rospy.is_shutdown():
        keys.publish()
        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, keys.old_attr)