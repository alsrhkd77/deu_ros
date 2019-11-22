#! /usr/bin/env python

import math
import numpy

import rospy
from std_msgs.msg import String

from Scan import Laser_Scan, Pose_Scan
from Drive import Drive_Method
import PosBag

class Exp(Laser_Scan, Pose_Scan):
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



class Explorer(Laser_Scan, Pose_Scan):
    def __init__(self):
        Laser_Scan.__init__(self)
        Pose_Scan.__init__(self)
        self.key_pub = rospy.Publisher('keys', String, queue_size=1)
        self.set_pose = False
        self.target_dir = 0.0
        self.stk_count = 0
        self.set_starting()

    def scan_callback(self, msg):
        Laser_Scan.scan_callback(self, msg)
        drive = Drive_Method()
        if PosBag.mode == 'Explorer':
            drive = Drive_Method()
            if self.set_pose:
                self.set_posture(drive) #set head poses
            else:
                self.expedition(drive)
        #self.print_explorer_status()

    def set_starting(self): #init status at starting
        self. target_dir = 90.0
        self.set_pose = True

    def set_posture(self, drive):
        turn_angle = self.rot_dir(self.angle, self.target_dir)
        if self.target_dir - 12.0 < round(self.angle, 2) < self.target_dir + 12.0:    #finish
            self.set_pose = False
            self.print_explorer_status()
            return #finish set
        elif self.target_dir - 29.0 < round(self.angle, 2) < self.target_dir + 29.0:    #turn 0.4
            if turn_angle >= 0:  # should turn left
                drive.smallTurnLeft()
            else:  # should turn left
                drive.smallTurnRight()
        elif self.target_dir - 45.0 < round(self.angle, 2) < self.target_dir + 45.0:  #turn 1.0
            if turn_angle >= 0:  # should turn left
                drive.angleTurnLeft()
            else:  # should turn left
                drive.angleTurnRight()
        else:   #turn 2.0 publish twice
            if turn_angle >= 0:  # should turn left
                drive.turnLeft()
                drive.publish()
                rospy.Rate(10).sleep()
            else:  # should turn left
                drive.turnRight()
                drive.publish()
                rospy.Rate(10).sleep()
        drive.publish()

    def expedition(self, drive):   #drive linear
        if self.range_ahead <= 0.6: #there's a wall in front
            self.set_pose = True
            drive.forceStop()
            drive.publish()
            self.check_side()
            pass    #todo: check side
            # return
        #check side path
        if self.left >= 1.5:
            pass
        if self.right >= 1.5:
            pass
        if self.left < 1.5 and self.right < 1.5:
            if round(self.left, 1) == round(self.right, 1):
                drive.increaseSpeed()
            elif self.left <= 0.5:
                drive.increaseRightTurn()
            elif self.right <= 0.5:
                drive.increaseLeftTurn()
            elif self.left < self.right:
                drive.increaseRightTurn()
            elif self.left > self.right:
                drive.increaseLeftTurn()
        else:
            drive.increaseSpeed()
        drive.publish()

    def check_side(self):
        now_angle = self.find_nearest(self.angle)
        check = ''
        if self.range_left > 1.5 and self.range_right > 1.5:
            check = 'both'
            self.target_dir = now_angle + 90.0  #turn right
        elif self.range_right > 1.5:
            check = 'right'
            self.target_dir = now_angle - 90.0  # turn right
        elif self.range_left > 1.5:
            check = 'left'
            self.target_dir = now_angle + 90.0  # turn left
        else:
            self.target_dir = now_angle - 180.0  # turn back

        #check overflow
        if self.target_dir >= 360:
            self.target_dir -= 360.0
        elif self.target_dir < 0:
            self.target_dir += 360.0

        self.set_pose = True

        #push stack and path_dic
        self.stk_count += 1
        PosBag.path[self.stk_count] = [self.stk_count - 1, self.position, check]
        PosBag.stk.append(self.stk_count)

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
        return round(angle, 2)

    def find_nearest(self, value):
        angle_array = numpy.array([0, 90, 180, 360])
        idx = (numpy.abs(angle_array - value)).argmin()
        return angle_array[idx]

    #TODO: erase print method
    def print_explorer_status(self):
        rospy.loginfo("angle: %0.10f", self.angle)
        rospy.loginfo("target_dir: %0.10f", self.target_dir)
        rospy.loginfo("range ahead: %0.10f", self.range_ahead)
        rospy.loginfo("range left: %0.10f", self.range_left)
        rospy.loginfo("range right %0.01f", self.range_right)
        rospy.loginfo("left: %0.10f", self.left)
        rospy.loginfo("right: %0.10f", self.right)