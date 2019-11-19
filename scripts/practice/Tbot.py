#! /usr/bin/env python

'''
Robot side
'''

import rospy

from Drive import Drive, Drive_Keys
from Scan import Scan
import PosBag
from Runner import Runner

if __name__ == "__main__":
    PosBag.temp()
    rospy.init_node('drive_bot')
    #scan = Scan()
    runner = Runner()
    drive_keys = Drive_Keys()
    rospy.spin()