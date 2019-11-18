#! /usr/bin/env python

import rospy

from Drive import Drive, Drive_Keys
from Scan import Scan, Pose_scan

if __name__ == "__main__":
    rospy.init_node('drive_bot')
    drive_keys = Drive_Keys()
    scan = Scan()
    pose = Pose_scan()
    rospy.spin()