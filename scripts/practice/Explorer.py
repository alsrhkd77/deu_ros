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
                self.set_posture(drive) #set head pose

    def set_starting(self): #init status at starting
        self. target_dir = 90.0

    def set_posture(self, drive):
        turn_angle = self.rot_dir(self.angle, self.target_dir)
        if self.target_dir - 45.0 < round(self.angle, 2) < self.target_dir + 45.0:
            if turn_angle >= 0:  # should turn left
                drive.angleTurnLeft()
            else:  # should turn left
                drive.angleTurnRight()
            pass  # turn 1.0
        elif self.target_dir - 29.0 < round(self.angle, 2) < self.target_dir + 29.0:
            if turn_angle >= 0:  # should turn left
                drive.smallTurnLeft()
            else:  # should turn left
                drive.smallTurnRight()
            pass  # turn 0.4
        elif self.target_dir - 12.0 < round(self.angle, 2) < self.target_dir + 12.0:
            self.set_pose = False
            return #finish set
        else:
            if turn_angle >= 0:  # should turn left
                drive.turnLeft()
            else:  # should turn left
                drive.turnRight()
        drive.publish()

    def expedition(self):   #drive linear
        pass

    def get_direction(self, position):
        angle = 180.0 * math.atan2(self.position_y - position[1], self.position_x - position[0]) / math.pi
        if angle < 0:
            angle += 360
        if angle >= 360:
            angle -= 360
        return round(angle, 2)

    def rot_dir(self, pre, target):
        pre = (pre * math.pi / 180) - math.pi
        target = (target * math.pi / 180) - math.pi
        angle = math.atan2(math.sin(target - pre), math.cos(target - pre)) * (180 / math.pi)
        return angle(angle, 2)