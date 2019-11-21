#! /usr/bin/env python

import math

import rospy
from std_msgs.msg import String

from Scan import Laser_Scan, Pose_Scan
from Drive import Drive_Method
import PosBag

class Explorer(Laser_Scan, Pose_Scan):
    def __init__(self):
        Laser_Scan.__init__(self)
        Pose_Scan.__init__(self)
        self.key_pub = rospy.Publisher('keys', String, queue_size=1)

    def scan_callback(self, msg):
        if PosBag.mode == 1:
            return
        Laser_Scan.scan_callback(self, msg)
        if PosBag.goal[0] - 0.5 < self.position_x < PosBag.goal[0] + 0.5 and PosBag.goal[1] - 0.5 < self.position_y < PosBag.goal[1] + 0.5:
            pass
        self.expedition()

    def expedition(self):
        pass


    def find_target(self):
        for l in list(reversed(PosBag.stk)):
            if 'both' in PosBag.path[l]:
                return l
        #todo return unused node
        return 0

    def getDirection(self, position):
        angle = 180.0 * math.atan2(self.position_y - position[1], self.position_x - position[0]) / math.pi
        if angle < 0:
            angle += 360
        if angle >= 360:
            angle -= 360
        return angle

class Exp(Laser_Scan, Pose_Scan):
    def __init__(self):
        Laser_Scan.__init__(self)
        Pose_Scan.__init__(self)
        self.key_pub = rospy.Publisher('keys', String, queue_size=1)
        self.set_pose = True
        self.target_dir = 0.0

    def scan_callback(self, msg):
        Laser_Scan.scan_callback(self, msg)
        if PosBag.mode == 'Explorer':
            drive = Drive_Method()
            if self.set_pose:
                self.set_posture(drive) #자세잡기

    def set_starting(self): #init status at starting
        self. target_dir = 90.0

    def set_posture(self, drive):
        drive.publish()

    def get_direction(self, position):
        angle = 180.0 * math.atan2(self.position_y - position[1], self.position_x - position[0]) / math.pi
        if angle < 0:
            angle += 360
        if angle >= 360:
            angle -= 360
        return angle

    def rot_dir(self, pre, target):
        pre = (pre * math.pi / 180) - math.pi
        target = (target * math.pi / 180) - math.pi
        result = math.atan2(math.sin(target - pre), math.cos(target - pre)) * (180 / math.pi)
        if result >= 0:
            pass #return  = turn right
        else:
            pass #return = turn left