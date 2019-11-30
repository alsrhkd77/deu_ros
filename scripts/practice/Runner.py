#! /usr/bin/env python
#-*-coding: utf-8-*-

import math
import numpy

import rospy

from Scan import Pose_Scan
from Drive import Drive_Method
import PosBag

class Runner(Pose_Scan):
    def __init__(self):
        Pose_Scan.__init__(self)
        self.start_check = True
        self.now_node = 0
        self.set_pose = True
        self.target_dir = 0.0

    def pose_callback(self, msg):
        Pose_Scan.pose_callback(self, msg)
        if PosBag.mode != 'Runner':  # runnig mode check
            return  # finish

        if self.start_check:
            self.start_check = False
            self.now_node = PosBag.stk.pop()
            self.set_pose = True
            self.target_dir = self.get_direction(PosBag.path[self.now_node][1])
        drive = Drive_Method()

        # check finish
        if -0.2 <= self.position_x <= 0.2 and -0.2 <= self.position_y <= 0.2:
            drive.forceStop()
            drive.publish()
            PosBag.mode = 'Finish'
            rospy.loginfo("Finish whole maze")
            print "now position: ", self.position
            return
        elif -0.5 <= self.position_x <= 0.5 and -0.5 <= self.position_y <= 0.5:
            start_point = [0,0]
            self.lead_to_target(drive, start_point)  # 도착점으로 타겟지정 + 이동
        else:
            if self.set_pose:
                self.set_posture(drive)
                return
            self.lead_to_target(drive, PosBag.path[self.now_node][1])

    def lead_to_target(self, drive, position):
        dir = self.find_nearest(self.angle)
        if (dir == 0 or dir == 360) and self.position[0] <= position[0]:
            drive.forceStop()
            drive.publish()
            self.get_next()
            return
        elif dir == 90 and self.position[1] <= position[1]:
            drive.forceStop()
            drive.publish()
            self.get_next()
            return
        elif dir == 180 and self.position[0] >= position[0]:
            drive.forceStop()
            drive.publish()
            self.get_next()
            return
        elif dir == 270 and self.position[1] >= position[1]:
            drive.forceStop()
            drive.publish()
            self.get_next()
            return
        else:
            target_dir = self.get_direction(position)
            turn_angle = self.rot_dir(self.angle, target_dir)
            drive.setTurn(turn_angle)
            drive.booster()
        drive.publish()

    def get_next(self):
        if len(PosBag.stk) > 0:
            self.now_node = PosBag.stk.pop()
        else:
            print "Nothing in PosBag.stk!"
        self.set_pose = True
        self.target_dir = self.get_direction(PosBag.path[self.now_node][1])

    def set_posture(self, drive):
        turn_angle = self.rot_dir(self.angle, self.target_dir)

        if self.target_dir == 0 and (round(self.angle) == 0 or round(self.angle) == 360):
            self.set_pose = False
            return  # finish set
        elif round(self.angle) == round(self.target_dir):    #finish
            self.set_pose = False
            return #finish set
        else:
            drive.forceTurn(turn_angle)
        drive.publish()

    def get_direction(self, position):  #get target direction
        angle = 180.0 * math.atan2(self.position_y - position[1], self.position_x - position[0]) / math.pi
        if angle < 0:
            angle += 360
        if angle >= 360:
            angle -= 360
        return round(angle, 2)

    def rot_dir(self, pre, target): #rotate which direction
        pre = (pre * math.pi / 180) - math.pi
        target = (target * math.pi / 180) - math.pi
        angle = math.atan2(math.sin(target - pre), math.cos(target - pre)) * (180 / math.pi)
        return angle

    def find_nearest(self, value):
        angle_array = numpy.array([0, 90, 180, 270, 360])
        idx = (numpy.abs(angle_array - value)).argmin()
        return angle_array[idx]