#! /usr/bin/env python

'''
Robot side
'''

import rospy

from Drive import Drive, Drive_Keys
from Scan import Scan

if __name__ == "__main__":
    rospy.init_node('drive_bot')
    scan = Scan()
    drive_keys = Drive_Keys()
    rospy.spin()