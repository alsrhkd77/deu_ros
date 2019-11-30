#! /usr/bin/env python

'''
Robot side
'''

import rospy

from Drive import Drive_Keys
from Explorer import Explorer
from Runner import Runner

if __name__ == "__main__":
    rospy.init_node('drive_bot')
    explorer = Explorer()
    runner = Runner()
    drive_keys = Drive_Keys()
    rospy.spin()