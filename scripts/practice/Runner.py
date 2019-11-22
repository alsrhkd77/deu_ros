#! /usr/bin/env python

import math

import rospy
from std_msgs.msg import String

from Scan import Laser_Scan, Pose_Scan
from Drive import Drive_Method
import PosBag

class Runner(Pose_Scan):
    def __init__(self):
        Pose_Scan.__init__(self)
        #self.mode = 0  # 0=Explorer, 1=Runner
        self.next = 0
        rospy.Subscriber('keys', String, self.keys_cb)

    def keys_cb(self, msg):
        if len(msg.data) == 0:
            return  # Unknown
        if msg.data[0] == 'v':
            self.mode = 1

    def pose_callback(self, msg):
        Pose_Scan.pose_callback(self, msg)
        if PosBag.mode == 'Explorer':
            return #finish
        if self.next == 0:
            self.next = PosBag.stk.pop()
        self.escape()

    def escape(self):
        node = PosBag.path[self.next]
        target_pose = node[1]
        drive = Drive_Method()
        target_dir = self.getDirection(target_pose)

        #if self.range_ahead <= 0.5 and self.position_x - 0.2 <= target_pose[0] <= self.position_x + 0.2 and self.position_y - 0.2 <= target_pose[1] <= self.position_y + 0.2:
        if self.position_x - 0.2 <= target_pose[0] <= self.position_x + 0.2 and self.position_y - 0.2 <= target_pose[1] <= self.position_y + 0.2:
            drive.forceStop()
            self.next = PosBag.stk.pop()
            drive.publish()
            return #finish


        print "target dir: ", target_dir
        print "angle: ", self.angle
        print "pose x: ", self.position_x
        print "pose y: ", self.position_y
        print "target pos x:", target_pose[0]
        print "target pos y:", target_pose[1]


        if target_dir <= 10 or target_dir >= 350:
            if 350.0 < self.angle or self.angle < target_dir + 10.0:  # match pose
                self.driveLinear(drive)
            else:  # set pose
                if self.angle > 180.0:
                    drive.angleTurnLeft()
                else:
                    drive.angleTurnRight()
        else:
            if target_dir - 10.0 < self.angle < target_dir + 10.0:  #match pose
                self.driveLinear(drive)
            else:   #set pose
                if self.angle - target_dir < 0:
                    drive.angleTurnLeft()
                else:
                    drive.angleTurnRight()
        drive.publish()

    def getDirection(self, position):
        angle = 180.0 * math.atan2(self.position_y - position[1], self.position_x - position[0]) / math.pi
        if angle < 0:
            angle += 360
        if angle > 360:
            angle -= 360
        return angle
        '''
        if (self.position_x - position[0]) < -1.5:
            return 270
        elif (self.position_x - position[0]) > 1.5:
            return 90
        elif (self.position_y - position[1]) < 1.5:
            return 0
        elif (self.position_y - position[1]) > 1.5:
            return 180
        '''

    def driveLinear(self, drive):
        drive.increaseSpeed()