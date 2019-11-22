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
        self.target_node = 0
        self.now_node = 0
        self.stk_count = 0
        self.set_starting()

    def scan_callback(self, msg):
        Laser_Scan.scan_callback(self, msg)
        drive = Drive_Method()
        if PosBag.mode == 'Explorer':
            drive = Drive_Method()
            if PosBag.goal[0] - 0.2 < round(self.position_x, 2) < PosBag.goal[0] + 0.2 and PosBag.goal[1] - 0.2 < round(
                    self.position_y, 2) < PosBag.goal[1] + 0.2:
                PosBag.mode = 'Runner'
                rospy.loginfo("Finish escape maze")
                print "position x: ", self.position_x
                print "position y: ", self.position_y
                print "path: ", PosBag.path
                print "stack: ", PosBag.stk
                return
            elif PosBag.goal[0] - 1.2 < round(self.position_x, 2) < PosBag.goal[0] + 1.2 and PosBag.goal[
                1] - 1.2 < round(self.position_y, 2) < PosBag.goal[1] + 1.2:
                self.target_dir = self.get_direction(PosBag.goal)
                self.set_pose = True
            elif round(self.position_y, 2) < -10:
                self.target_dir = self.get_direction(PosBag.goal)
                self.set_pose = True
            node_x = PosBag.path[self.now_node][1][0]
            node_y = PosBag.path[self.now_node][1][1]
            if node_x - 0.5 < round(self.position_x, 2) < node_x + 0.5 \
                    and node_y - 0.5 < round(self.position_y, 2) < node_y + 0.5 \
                    and self.target_node != 0:
                self.roll_back()

            if self.set_pose:
                self.set_posture(drive) #set head poses
            else:
                self.expedition(drive)
        #self.print_explorer_status()


    def pose_callback(self, msg):
        Pose_Scan.pose_callback(self, msg)

        if not PosBag.mode == 'Explorer':
            return

        '''
        if PosBag.goal[0] - 0.2 < round(self.position_x, 2) < PosBag.goal[0] + 0.2 and PosBag.goal[1] - 0.2 < round(self.position_y, 2) < PosBag.goal[1] + 0.2:
            PosBag.mode = 'Runner'
            rospy.loginfo("Finish escape maze")
            print "position x: ", self.position_x
            print "position y: ", self.position_y
            print "path: ", PosBag.path
            print "stack: ", PosBag.stk
            return
        elif PosBag.goal[0] - 1.2 < round(self.position_x, 2) < PosBag.goal[0] + 1.2 and PosBag.goal[1] - 1.2 < round(self.position_y, 2) < PosBag.goal[1] + 1.2:
            self.target_dir = self.get_direction(PosBag.goal)
            self.set_pose = True
            return
        elif round(self.position_y, 2) < -10:
            self.target_dir = self.get_direction(PosBag.goal)
            self.set_pose = True
            return
        node_x = PosBag.path[self.now_node][1][0]
        node_y = PosBag.path[self.now_node][1][1]
        if node_x - 0.5 < round(self.position_x, 2) < node_x + 0.5\
                and node_y - 0.5 < round(self.position_y, 2) < node_y + 0.5\
                and self.target_node != 0:
            self.roll_back()
        '''

    def roll_back(self):
        if self.now_node == self.target_node:
            PosBag.stk.append(self.now_node)
            self.set_pose = False
            self.target_node = 0
            post = PosBag.stk[PosBag.stk.index(self.now_node) - 1]
            self.target_dir = PosBag.path[post][3]
            #print "old angle: ", PosBag.path[self.target_node][3] - 90.0
            if self.target_dir >= 360.0:
                self.target_dir -= 360
            elif self.target_dir < 0:
                self.target_dir += 360
            self.old_target_dir = self.target_dir
            return  #finish roll back
        else:
            # arrived go to next node
            self.now_node = PosBag.stk.pop()
            self.target_dir = self.get_direction(PosBag.path[self.now_node][1])
            self.set_pose = True
        #self.print_explorer_status()
        #print "path: ", PosBag.path
        #print "stack: ", PosBag.stk

    def set_starting(self): #init status at starting
        self.target_dir = 90.0
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
            #self.print_explorer_status()
            self.set_pose = True
            drive.forceStop()
            drive.publish()
            if self.target_node == 0:
                self.check_side()
            else:
                self.roll_back()
                #self.target_dir = self.get_direction(PosBag.path[self.now_node][1])
                #self.set_pose = True
            return

        #check side path
        if self.range_left_min >= 1.5 and self.left_wall == [0, 0]:
            self.left_wall = self.position
        elif not(self.range_left_min >= 1.5) and not(self.left_wall == [0, 0]):
            position = numpy.round((numpy.array(self.position) + numpy.array(self.left_wall)) / 2, 2)
            self.push_unused(position.tolist(), 'left')
            self.left_wall = [0, 0]

        if self.range_right_min >= 1.5 and self.right_wall == [0, 0]:
            self.right_wall = self.position
        elif not(self.range_right_min >= 1.5) and not(self.right_wall == [0, 0]):
            position = numpy.round((numpy.array(self.position) + numpy.array(self.left_wall)) / 2, 2)
            numpy.round(position, 2)
            self.push_unused(position.tolist(), 'right')
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
            drive = Drive_Method()
            # roll_back start
            self.target_node = self.find_target()
            self.now_node = self.stk_count
            self.roll_back()
            #self.target_dir = self.get_direction(PosBag.path[self.now_node][1])  # turn back
            #self.print_explorer_status()

        #check overflow
        if self.target_dir >= 360:
            self.target_dir -= 360.0
        elif self.target_dir < 0:
            self.target_dir += 360.0
        #self.print_explorer_status()
        self.set_pose = True

        if not check == '':
            #push stack and path_dic
            self.push_bag(self.position, check)

    def push_bag(self, position, dir):  #push stack and path_dic
        self.stk_count += 1

        if dir == 'both':
            #check stack
            PosBag.stk.reverse()
            for i in PosBag.stk:
                if position[0] - 1.0 <= PosBag.path[i][1][0] <= position[0] + 1.0 and position[1] - 1.0 <= PosBag.path[i][1][1] <= position[1] + 1.0:
                    PosBag.stk.remove(i)
            PosBag.stk.reverse()

        PosBag.path[self.stk_count] = [self.stk_count - 1, position, dir, self.target_dir]
        PosBag.stk.append(self.stk_count)
        #print "path: ", PosBag.path
        #print "stack: ", PosBag.stk

    def push_unused(self, position, dir):
        self.stk_count += 1
        PosBag.path[self.stk_count] = [self.stk_count - 1, position, dir, self.target_dir]
        PosBag.unused.append(self.stk_count)
        #print "path: ", PosBag.path
        #print "stack: ", PosBag.stk

    def get_direction(self, position):  #get target direction
        angle = 180.0 * math.atan2(self.position_y - position[1], self.position_x - position[0]) / math.pi
        if angle < 0:
            angle += 360
        if angle >= 360:
            angle -= 360
        return round(angle, 2)
        #return round(angle, 2)

    def rot_dir(self, pre, target): #rotate which direction
        pre = (pre * math.pi / 180) - math.pi
        target = (target * math.pi / 180) - math.pi
        angle = math.atan2(math.sin(target - pre), math.cos(target - pre)) * (180 / math.pi)
        return round(angle, 2)

    def find_nearest(self, value):
        angle_array = numpy.array([0, 90, 180, 360])
        idx = (numpy.abs(angle_array - value)).argmin()
        return angle_array[idx]

    def find_target(self):
        for l in list(reversed(PosBag.stk)):
            if 'both' in PosBag.path[l]:
                return l
        # return unused node
        if len(PosBag.unused) != 0:
            node = PosBag.unused.pop()
            del PosBag.stk[1:]
            PosBag.stk.append(node)
            return node
        else:
            rospy.loginfo("PosBag.unused stack is empty!")
        return 0

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
        rospy.loginfo("target node %0.01f" %self.target_node)
        rospy.loginfo("now node %0.01f" %self.now_node)