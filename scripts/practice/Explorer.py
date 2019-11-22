#! /usr/bin/env python

import math
import numpy

import rospy
from std_msgs.msg import String

from Scan import Laser_Scan, Pose_Scan
from Drive import Drive_Method
import PosBag

import Drive_vel    #remove at last

'''
not use this class, have to be erased!
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
'''


class Explorer(Laser_Scan, Pose_Scan):
    def __init__(self):
        Laser_Scan.__init__(self)
        Pose_Scan.__init__(self)
        self.key_pub = rospy.Publisher('keys', String, queue_size=1)
        self.set_pose = False
        self.old_target_dir = 0.0
        self.left_wall = [0, 0]
        self.right_wall = [0, 0]
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

    def pose_callback(self, msg):
        Pose_Scan.pose_callback(self, msg)

    def roll_back(self, drive):
        pass

    def set_starting(self): #init status at starting
        self. target_dir = 90.0
        self.old_target_dir = self.angle
        self.set_pose = True

    def set_posture(self, drive):
        turn_angle = self.rot_dir(self.angle, self.target_dir)
        drive.forceStop()
        if self.target_dir - 12.0 < round(self.angle, 2) < self.target_dir + 12.0:    #finish
            self.set_pose = False
            self.old_target_dir = self.target_dir
            #self.print_explorer_status()
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
        max_margin = 12.0
        if self.range_ahead <= 0.6: #there's a wall in front
            self.set_pose = True
            drive.forceStop()
            drive.publish()
            self.check_side()
            return

        #check side path
        if self.range_left_min >= 1.5 and self.left_wall == [0, 0]:
            self.left_wall = self.position
        elif not(self.range_left_min >= 1.5) and not(self.left_wall == [0, 0]):
            position = numpy.round((numpy.array(self.position) + numpy.array(self.left_wall)) / 2, 2)
            self.push_bag(position.tolist(), 'left')
            self.left_wall = [0, 0]

        if self.range_right_min >= 1.5 and self.right_wall == [0, 0]:
            self.right_wall = self.position
        elif not(self.range_right_min >= 1.5) and not(self.right_wall == [0, 0]):
            position = numpy.round((numpy.array(self.position) + numpy.array(self.left_wall)) / 2, 2)
            numpy.round(position, 2)
            self.push_bag(position.tolist(), 'right')
            self.right_wall = [0, 0]

        if self.range_left_min < 1.5 and self.range_right_min < 1.5:
            if round(self.range_left_min, 1) == round(self.range_right_min, 1):
                drive.increaseSpeed()
            elif self.target_dir == 0 and (self.angle > max_margin or self.angle < 360 - max_margin):
                if self.angle > 180.0:
                    drive.smallTurnLeft()
                    drive.booster()
                else:
                    drive.smallTurnRight()
                    drive.booster()
            elif not(self.target_dir - max_margin < self.angle < self.target_dir + max_margin):
                if self.target_dir + max_margin > self.angle:
                    drive.smallTurnLeft()
                    drive.booster()
                else:
                    drive.smallTurnRight()
                    drive.booster()
            elif self.range_left_min <= 0.5:
                #drive.increaseRightTurn()
                drive.smallTurnRight()
                drive.booster()
            elif self.range_right_min <= 0.5:
                #drive.increaseLeftTurn()
                drive.smallTurnLeft()
                drive.booster()
            elif self.range_left_min < self.range_right_min:
                #drive.increaseRightTurn()
                drive.smallTurnRight()
                drive.booster()
            elif self.range_left_min > self.range_right_min:
                #drive.increaseLeftTurn()
                drive.smallTurnLeft()
                drive.booster()
        else:
            drive.increaseSpeed()
        drive.publish()

    def check_side(self):
        self.left_wall = [0, 0]
        self.right_wall = [0, 0]
        #now_angle = self.find_nearest(self.angle)
        check = ''
        if self.range_left_max > 1.5 and self.range_right_max > 1.5:
            check = 'both'
            self.target_dir = self.old_target_dir + 90.0  #turn right
        elif self.range_right_max > 1.5:
            check = 'right'
            self.target_dir = self.old_target_dir - 90.0  # turn right
        elif self.range_left_max > 1.5:
            check = 'left'
            self.target_dir = self.old_target_dir + 90.0  # turn left
        else:
            self.target_dir = self.old_target_dir - 180.0  # turn back

        #check overflow
        if self.target_dir >= 360:
            self.target_dir -= 360.0
        elif self.target_dir < 0:
            self.target_dir += 360.0
        #self.print_explorer_status()
        self.set_pose = True

        #push stack and path_dic
        self.push_bag(self.position, check)

    def push_bag(self, position, dir):  #push stack and path_dic
        if dir == 'both':
            #check stack
            PosBag.stk.reverse()
            for i in PosBag.stk:
                if position[0] - 1.0 <= PosBag.path[i][1][0] <= position[0] + 1.0 and position[1] - 1.0 <= PosBag.path[i][1][1] <= position[1] + 1.0:
                    PosBag.stk.remove(i)
            PosBag.stk.reverse()
        self.stk_count += 1
        PosBag.path[self.stk_count] = [self.stk_count - 1, position, dir]
        PosBag.stk.append(self.stk_count)
        print "path: ", PosBag.path
        print "stack: ", PosBag.stk

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
        rospy.loginfo("angle: %0.10f" %self.angle)
        rospy.loginfo("target_dir: %0.10f" %self.target_dir)
        rospy.loginfo("range ahead: %0.10f" %self.range_ahead)
        rospy.loginfo("range left min: %0.1f" % self.range_left_min)
        rospy.loginfo("range right min: %0.1f" % self.range_right_min)
        rospy.loginfo("range left max: %0.1f" % self.range_left_max)
        rospy.loginfo("range right max: %0.1f" % self.range_right_max)
        rospy.loginfo("left: %0.10f" %self.left)
        rospy.loginfo("right: %0.10f" %self.right)
        rospy.loginfo("speed: %0.01f" %Drive_vel.speed)
        rospy.loginfo("turn: %0.01f" %Drive_vel.turn)