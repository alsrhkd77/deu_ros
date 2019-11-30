#! /usr/bin/env python
#-*-coding: utf-8-*-

import math
import numpy

import rospy

from Scan import Laser_Scan, Pose_Scan
from Drive import Drive_Method
import PosBag

class Explorer(Laser_Scan, Pose_Scan):
    def __init__(self):
        Laser_Scan.__init__(self)   # call parent's init method
        Pose_Scan.__init__(self)    # call parent's init method
        self.left_wall = [0, 0]
        self.right_wall = [0, 0]
        self.set_pose = False
        self.target_node = 0
        self.now_node = 0
        self.stk_count = 0
        self.target_dir = 0.0
        self.set_starting()

    def pose_callback(self, msg):
        Pose_Scan.pose_callback(self, msg)

    def scan_callback(self, msg):
        Laser_Scan.scan_callback(self, msg)
        if PosBag.mode != 'Explorer':   #runnig mode check
            return #finish
        drive = Drive_Method()

        #check finish
        if PosBag.goal[0] - 0.2 <= self.position_x <= PosBag.goal[0] + 0.2 and PosBag.goal[1] - 0.2 <= self.position_y <= PosBag.goal[1] + 0.2:
            drive.forceStop()
            drive.publish()
            PosBag.mode = 'Runner'
            rospy.loginfo("Finish escape maze")
            print "now position: ", self.position
            return
        elif PosBag.goal[0] - 0.5 <= self.position_x <= PosBag.goal[0] + 0.5 and PosBag.goal[1] - 0.5 <= self.position_y <= PosBag.goal[1] + 0.5:
            self.lead_to_target(drive, PosBag.goal)    #도착점으로 타겟지정 + 이동
        else:
            if self.set_pose:
                self.set_posture(drive) # 로봇 방향 잡기
                return
            if self.target_node != 0:
                self.lead_to_target(drive, PosBag.path[self.now_node][1])   # 목표지점을 향해 이동
            else:
                self.expedition(drive)  # 일단 탐색 이동

    def expedition(self, drive):   #drive linear
        if self.range_ahead <= 0.65:    # there's wall in ahead
            drive.forceStop()
            drive.publish()
            self.check_side()
            return
        elif self.range_left_min < 1.5 and self.range_right_min < 1.5:  # check close to the side wall
            if self.range_left_min <= 0.4:
                drive.smallTurnRight()
                drive.publish()
                return
            elif self.range_right_min <= 0.4:
                drive.smallTurnLeft()
                drive.publish()
                return

        # check side path(left)
        if self.range_left_max >= 2.5 and self.left_wall == list([0, 0]):
            self.left_wall = self.position
        elif not (self.range_left_max >= 2.5) and self.left_wall != list([0, 0]) and self.target_node == 0:
            position = numpy.round((numpy.array(self.position) + numpy.array(self.left_wall)) / 2, 2)
            numpy.round(position, 2)
            self.push_unused(position.tolist(), 'left')
            self.left_wall = [0, 0]

        # check side path(right)
        if self.range_right_max >= 2.5 and self.right_wall == list([0, 0]):
            self.right_wall = self.position
        elif not (self.range_right_max >= 2.5) and self.right_wall != list([0, 0]) and self.target_node == 0:
            position = numpy.round((numpy.array(self.position) + numpy.array(self.right_wall)) / 2, 2)
            numpy.round(position, 2)
            self.push_unused(position.tolist(), 'right')
            self.right_wall = [0, 0]

        # make corrections
        if self.angle != self.target_dir:
            turn_angle = self.rot_dir(self.angle, self.target_dir)
            drive.setTurn(turn_angle)
            drive.booster()
        else:
            drive.fullSpeed()
        drive.publish()

    def lead_to_target(self, drive, position):
        dir = self.find_nearest(self.angle)
        if (dir == 0 or dir == 360) and self.position[0] <= position[0]:
            drive.forceStop()
            drive.publish()
            self.roll_back()
            return
        elif dir == 90 and self.position[1] <= position[1]:
            drive.forceStop()
            drive.publish()
            self.roll_back()
            return
        elif dir == 180 and self.position[0] >= position[0]:
            drive.forceStop()
            drive.publish()
            self.roll_back()
            return
        elif dir == 270 and self.position[1] >= position[1]:
            drive.forceStop()
            drive.publish()
            self.roll_back()
            return
        else:
            target_dir = self.get_direction(position)
            turn_angle = self.rot_dir(self.angle, target_dir)
            drive.setTurn(turn_angle)
            drive.booster()
        drive.publish()

    def get_direction(self, position):  #get target direction
        angle = 180.0 * math.atan2(self.position_y - position[1], self.position_x - position[0]) / math.pi
        if angle < 0:
            angle += 360
        if angle >= 360:
            angle -= 360
        return angle

    def find_nearest(self, value):
        angle_array = numpy.array([0, 90, 180, 270, 360])
        idx = (numpy.abs(angle_array - value)).argmin()
        return angle_array[idx]

    def rot_dir(self, pre, target): #rotate which direction
        pre = (pre * math.pi / 180) - math.pi
        target = (target * math.pi / 180) - math.pi
        angle = math.atan2(math.sin(target - pre), math.cos(target - pre)) * (180 / math.pi)
        return angle

    def set_starting(self): #init status at starting
        self. target_dir = 90.0
        self.set_pose = True

    def set_posture(self, drive):
        self.left_wall = [0, 0]
        self.right_wall = [0, 0]
        turn_angle = self.rot_dir(self.angle, self.target_dir)

        if self.target_dir == 0 and (round(self.angle) == 0 or round(self.angle) == 360):
            self.set_pose = False
            return  # finish set
        elif round(self.angle) == round(self.target_dir):
            self.set_pose = False
            return  # finish set
        else:
            drive.forceTurn(turn_angle)
        drive.publish()

    def check_side(self):
        self.left_wall = [0, 0]
        self.right_wall = [0, 0]
        self.set_pose = True
        check = ''
        if self.range_left_max > 2.0 and self.range_right_max > 2.0:
            check = 'both'
            self.target_dir = self.target_dir + 90.0  #turn right / 변경시 rollback도 함께 변경
        elif self.range_right_max > 2.0:
            check = 'right'
            self.target_dir = self.target_dir - 90.0  # turn right
        elif self.range_left_max > 2.0:
            check = 'left'
            self.target_dir = self.target_dir + 90.0  # turn left
        else:
            self.target_dir = self.target_dir - 180.0  # turn back
            # check overflow
            if self.target_dir >= 360:
                self.target_dir -= 360.0
            elif self.target_dir < 0:
                self.target_dir += 360.0
            self.target_dir = self.find_nearest(self.target_dir)
            self.target_node = self.find_target()
            self.roll_back()
            return

        #check overflow
        if self.target_dir >= 360:
            self.target_dir -= 360.0
        elif self.target_dir < 0:
            self.target_dir += 360.0
        self.target_dir = self.find_nearest(self.target_dir)

        #push stack and path_dic
        self.push_bag(self.position, check)

    def roll_back(self):
        self.set_pose = True
        if self.now_node == self.target_node:
            PosBag.stk.append(self.now_node)
            self.target_node = 0
            if PosBag.path[self.now_node][3] == 'left':
                self.target_dir = PosBag.path[self.now_node][3] + 90
            elif PosBag.path[self.now_node][3] == 'right':
                self.target_dir = PosBag.path[self.now_node][3] - 90
            else:
                self.target_dir = PosBag.path[self.now_node][3] - 90
            if self.target_dir >= 360.0:
                self.target_dir -= 360
            elif self.target_dir < 0:
                self.target_dir += 360
            self.target_dir = self.find_nearest(self.target_dir)
            self.now_node = 0
            return  # finish roll back
        else:
            # arrived go to next node
            self.now_node = PosBag.stk.pop()
            self.target_dir = self.get_direction(PosBag.path[self.now_node][1])
            if self.target_dir >= 360.0:
                self.target_dir -= 360
            elif self.target_dir < 0:
                self.target_dir += 360

    def push_bag(self, position, dir):  #push stack and path_dic
        if dir == 'both':
            for i in reversed(PosBag.unused):   #check stack
                if position[0] - 1.0 <= PosBag.path[i][1][0] <= position[0] + 1.0 and position[1] - 1.0 <= PosBag.path[i][1][1] <= position[1] + 1.0:
                    PosBag.unused.remove(i)
        self.stk_count += 1
        angle = self.find_nearest(self.angle)
        PosBag.path[self.stk_count] = [self.stk_count - 1, position, dir, angle]
        PosBag.stk.append(self.stk_count)

    def push_unused(self, position, dir):  #push stack and path_dic
        check = True
        #check path dic
        for i in PosBag.path.keys():
            if position[0] - 1.0 <= PosBag.path[i][1][0] <= position[0] + 1.0 and position[1] - 1.0 <= PosBag.path[i][1][1] <= position[1] + 1.0:
                check = False
                break

        #push node
        if check:
            self.stk_count += 1
            PosBag.path[self.stk_count] = [self.stk_count - 1, position, dir, self.angle]
            PosBag.unused.append(self.stk_count)

    def find_target(self):
        for node in reversed(PosBag.stk):
            if 'both' == PosBag.path[node][2]:
                return node
            elif len(PosBag.unused) > 0 and node < PosBag.unused[-1]:
                u_node = PosBag.unused.pop()
                del PosBag.stk[PosBag.stk.index(node) + 1:]
                PosBag.stk.append(u_node)
                PosBag.stk.sort()
                return u_node
        # return unused node
        if len(PosBag.unused) != 0:
            node = PosBag.unused.pop()
            del PosBag.stk[1:]
            PosBag.stk.append(node)
            return node
        else:
            rospy.loginfo("Whole Stack is Empty!\nPath list: ", PosBag.path)
            return 0