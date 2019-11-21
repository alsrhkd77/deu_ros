#! /usr/bin/env python

'''
Robot side
'''

import rospy

from Drive import Drive_Keys
from Scan import Scan
import PosBag
from Explorer import Explorer
from Runner import Runner

if __name__ == "__main__":
    #PosBag.temp()
    #PosBag.stk.append(0)
    rospy.init_node('drive_bot')
    scan = Scan()
    #explorer = Explorer()
    #runner = Runner()
    drive_keys = Drive_Keys()
    rospy.spin()