#! /usr/bin/env python
#-*-coding: utf-8-*-

import math
import numpy

import rospy
from std_msgs.msg import String

from Scan import Laser_Scan, Pose_Scan
from Drive import Drive_Method
import PosBag

import Drive_vel    #remove at last


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
        self.target_node = 0
        self.now_node = 0
        self.set_starting()

    def scan_callback(self, msg):
        Laser_Scan.scan_callback(self, msg)
        if PosBag.mode == 'Explorer':
            drive = Drive_Method()
            if PosBag.goal[0] - 0.2 < self.position_x < PosBag.goal[0] + 0.2 and PosBag.goal[1] - 0.2 < self.position_y < PosBag.goal[1] + 0.2:
                drive.forceStop()
                drive.publish()
                PosBag.mode = 'Runner'
                rospy.loginfo("Finish escape maze")
                return
            elif PosBag.goal[0] - 1.2 < self.position_x < PosBag.goal[0] + 1.2 and PosBag.goal[1] - 1.2 < self.position_y < PosBag.goal[1] + 1.2:
                self.target_dir = self.get_direction(PosBag.goal)
                if self.target_dir - 5.0 < self.angle < self.target_dir + 5.0:
                    self.set_pose = True
            elif self.position_y < -10:
                self.target_dir = self.get_direction(PosBag.goal)
                if self.target_dir - 5.0 < self.angle < self.target_dir + 5.0:
                    self.set_pose = True
            self.print_explorer_status()

            node_x = PosBag.path[self.now_node][1][0]
            node_y = PosBag.path[self.now_node][1][1]
            if node_x - 0.2 <= self.position_x <= node_x + 0.2 \
                    and node_y - 0.2 <= self.position_y <= node_y + 0.2 \
                    and self.target_node != 0:
                drive.forceStop()
                drive.publish()
                self.roll_back()

            if self.set_pose:
                self.set_posture(drive) #set head poses
            else:
                self.expedition(drive)

    def pose_callback(self, msg):
        Pose_Scan.pose_callback(self, msg)

    def roll_back(self):
        if self.now_node == self.target_node:
            PosBag.stk.append(self.now_node)
            self.set_pose = False
            self.target_node = 0
            #post = PosBag.stk[PosBag.stk.index(self.now_node) - 1]
            if PosBag.path[self.now_node][3] == 'left':
                self.target_dir = PosBag.path[self.now_node][3] + 90
            elif PosBag.path[self.now_node][3] == 'right':
                self.target_dir = PosBag.path[self.now_node][3] - 90
            else:
                self.target_dir = PosBag.path[self.now_node][3] - 90
            # print "old angle: ", PosBag.path[self.target_node][3] - 90.0
            if self.target_dir >= 360.0:
                self.target_dir -= 360
            elif self.target_dir < 0:
                self.target_dir += 360
            self.target_dir = self.find_nearest(self.target_dir)
            self.now_node = 0
            #self.old_target_dir = self.target_dir
            return  # finish roll back
        else:
            # arrived go to next node
            self.now_node = PosBag.stk.pop()
            self.target_dir = self.get_direction(PosBag.path[self.now_node][1])
            self.set_pose = True
            if self.target_dir >= 360.0:
                self.target_dir -= 360
            elif self.target_dir < 0:
                self.target_dir += 360
        # self.print_explorer_status()
        # print "path: ", PosBag.path
        # print "stack: ", PosBag.stk

    def set_starting(self): #init status at starting
        self. target_dir = 90.0
        self.old_target_dir = self.angle
        self.set_pose = True

    def set_posture(self, drive):
        self.left_wall = [0, 0]
        self.right_wall = [0, 0]
        turn_angle = self.rot_dir(self.angle, self.target_dir)

        if self.target_dir == 0 and (self.angle < 5.0 or self.angle > 360 - 5.0):
            self.set_pose = False
            self.old_target_dir = self.target_dir
            # self.print_explorer_status()
            return  # finish set
        elif self.target_dir - 5.0 < round(self.angle, 2) < self.target_dir + 5.0:    #finish
            self.set_pose = False
            self.old_target_dir = self.target_dir
            #self.print_explorer_status()
            return #finish set
        else:
            drive.forceTurn(turn_angle)
        drive.publish()


    def expedition(self, drive):   #drive linear
        max_margin = 5.0
        if self.range_ahead < 0.6: #there's a wall in front
            self.left_wall = [0, 0]
            self.right_wall = [0, 0]
            self.set_pose = True
            drive.forceStop()
            drive.publish()
            if self.target_node == 0:
                self.check_side()
            else:
                self.roll_back()
            return

        #check side path
        if self.range_left_max >= 2.0 and self.left_wall == list([0, 0]):
            self.left_wall = self.position
        elif not(self.range_left_max >= 2.0) and self.left_wall != list([0, 0]) and self.target_node == 0:
            position = numpy.round((numpy.array(self.position) + numpy.array(self.left_wall)) / 2, 2)
            self.push_unused(position.tolist(), 'left')
            self.left_wall = [0, 0]

        if self.range_right_max >= 2.0 and self.right_wall == list([0, 0]):
            self.right_wall = self.position
        elif not(self.range_right_max >= 2.0) and self.right_wall != list([0, 0]) and self.target_node == 0:
            position = numpy.round((numpy.array(self.position) + numpy.array(self.left_wall)) / 2, 2)
            numpy.round(position, 2)
            self.push_unused(position.tolist(), 'right')
            self.right_wall = [0, 0]

        '''
        if self.target_node != 0:
            self.target_dir = self.get_direction(PosBag.path[self.now_node][1])
        '''

        turn_angle = self.rot_dir(self.angle, self.target_dir)
        print "turn_angle: ", turn_angle

        if self.range_left_min < 1.5 and self.range_right_min < 1.5:
            if round(self.range_left_min, 1) == round(self.range_right_min, 1):
                drive.increaseSpeed()
            elif self.target_dir == 0 and (self.angle > max_margin or self.angle < 360 - max_margin):
                if self.angle > 180.0:
                    #drive.smallTurnLeft()
                    drive.setTurn(turn_angle)
                    drive.booster()
                else:
                    #drive.smallTurnRight()
                    drive.setTurn(turn_angle)
                    drive.booster()
            elif not(self.target_dir - max_margin < self.angle < self.target_dir + max_margin):
                if self.target_dir + max_margin > self.angle:
                    #drive.smallTurnLeft()
                    drive.setTurn(turn_angle)
                    drive.booster()
                else:
                    #drive.smallTurnRight()
                    drive.setTurn(turn_angle)
                    drive.booster()
            elif self.range_left_min <= 0.5:
                #drive.increaseRightTurn()
                drive.smallTurnRight()
                drive.booster()
            elif self.range_right_min <= 0.5:
                #drive.increaseLeftTurn()
                drive.smallTurnLeft()
                drive.booster()
            else:
                drive.increaseSpeed()
        else:
            drive.increaseSpeed()
        drive.publish()

    def check_side(self):
        self.left_wall = [0, 0]
        self.right_wall = [0, 0]
        #now_angle = self.find_nearest(self.angle)
        check = ''
        if self.range_left_max > 2.0 and self.range_right_max > 2.0:
            check = 'both'
            self.target_dir = self.old_target_dir + 90.0  #turn right
        elif self.range_right_max > 2.0:
            check = 'right'
            self.target_dir = self.old_target_dir - 90.0  # turn right
        elif self.range_left_max > 2.0:
            check = 'left'
            self.target_dir = self.old_target_dir + 90.0  # turn left
        else:
            self.target_dir = self.old_target_dir - 180.0  # turn back
            self.target_node = self.find_target()
            #self.now_node = PosBag.stk.pop()
            self.roll_back()
            return

        #check overflow
        if self.target_dir >= 360:
            self.target_dir -= 360.0
        elif self.target_dir < 0:
            self.target_dir += 360.0
        #self.print_explorer_status()
        self.set_pose = True
        self.target_dir = self.find_nearest(self.target_dir)

        #push stack and path_dic
        self.push_bag(self.position, check)

    def push_bag(self, position, dir):  #push stack and path_dic
        if dir == 'both':
            #check stack
            for i in reversed(PosBag.unused):
                if position[0] - 1.0 <= PosBag.path[i][1][0] <= position[0] + 1.0 and position[1] - 1.0 <= PosBag.path[i][1][1] <= position[1] + 1.0:
                    PosBag.unused.remove(i)
        self.stk_count += 1
        PosBag.path[self.stk_count] = [self.stk_count - 1, position, dir, self.angle]
        PosBag.stk.append(self.stk_count)
        '''
        print "path: ", PosBag.path
        print "stack: ", PosBag.stk
        print "unused: ", PosBag.unused
        '''

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

    def find_target(self):
        for node in reversed(PosBag.stk):
            if 'both' in PosBag.path[node]:
                return node
            elif len(PosBag.unused) > 0 and node > PosBag.unused[-1]:
                u_node = PosBag.unused.pop()
                del PosBag.stk[node:]
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

    def find_nearest(self, value):
        angle_array = numpy.array([0, 90, 180, 270, 360])
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
        rospy.loginfo("target_node: %d" %self.target_node)
        rospy.loginfo("now_node: %d" %self.now_node)


class exp(Laser_Scan, Pose_Scan):
    def __init__(self):
        Laser_Scan.__init__(self)
        Pose_Scan.__init__(self)

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
            return
        elif PosBag.goal[0] - 0.5 <= self.position_x <= PosBag.goal[0] + 0.5 and PosBag.goal[1] - 0.5 <= self.position_y <= PosBag.goal[1] + 0.5:
            pass    #도착점으로 타겟지정 + 이동

    def expedition(self, drive):   #drive linear
        pass