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
        self.set_pose = False
        self.count_node = 0
        self.next = 0
        self.target_node = 0
        self.target_dir = 90.0
        self.leftStartPos = [0.0, 0.0]
        self.rightStartPos = [0.0, 0.0]

    def scan_callback(self, msg):
        Laser_Scan.scan_callback(self, msg)
        if PosBag.goal[0] - 0.2 <= self.position_x <= PosBag.goal[0] + 0.2 and PosBag.goal[1] - 0.2 <= self.position_y <= PosBag.goal[1] + 0.2:
            self.key_pub.publish('v')
            return #finish
        elif self.position_y < -5.0:
            # push node
            self.count_node += 1
            postnode = PosBag.stk[len(PosBag.stk) - 1]
            PosBag.path[self.count_node] = [postnode, PosBag.goal, None, 270]
            PosBag.stk.append(self.count_node)
            # print PosBag.path.items()
            self.escape()
        elif self.target_node != 0:
            self.escape()
        else:
            self.expedition()
        #print PosBag.path.items()

    def turnCheck(self, drive):
        #check path
        dir = ''
        '''
        if self.left >= 1.5 and self.right >= 1.5:
            dir = 'both'
            self.target_dir -= 90.0
            drive.angleTurnRight()
        elif self.left >= 1.5:
            dir = 'left'
            self.target_dir += 90.0
            drive.angleTurnLeft()
        elif self.right >= 1.5:
            dir = 'right'
            self.target_dir -= 90.0
            drive.angleTurnRight()
        else:
            self.target_dir -= 180.0
            # check angle
            if self.target_dir > 360:
                self.target_dir -= 360
            if self.target_dir < 0:
                self.target_dir += 360
            drive.angleTurnRight()
            self.target_node = 0 #todo!!!
            return
        '''
        '''
        print "range ahead: %0.1f" % self.range_ahead
        print "range left: %0.1f" % self.range_left
        print "range right: %0.1f" % self.range_right
        print "left: %0.1f" % self.left
        print "right: %0.1f" % self.right
        '''
        if self.set_pose == False and self.range_left >= 1.5 and self.range_right >= 1.5:
            self.set_pose = True
            dir = 'both'
            self.target_dir -= 90.0
            drive.forceStop()
            drive.angleTurnRight()
        elif self.set_pose == False and self.range_left >= 1.5:
            self.set_pose = True
            dir = 'left'
            self.target_dir += 90.0
            drive.forceStop()
            drive.angleTurnLeft()
        elif self.set_pose == False and self.range_right >= 1.5:
            self.set_pose = True
            dir = 'right'
            self.target_dir -= 90.0
            drive.forceStop()
            drive.angleTurnRight()
        elif self.set_pose == False and self.range_left < 1.5 and self.range_right < 1.5:
            #self.set_pose = True
            self.target_dir -= 180.0
            # check angle
            if self.target_dir >= 360:
                self.target_dir -= 360
            if self.target_dir < 0:
                self.target_dir += 360
            drive.forceStop()
            drive.angleTurnRight()
            self.target_node = self.find_target()
            self.next = self.count_node
            return


        #check angle
        if self.target_dir >= 360.0:
            self.target_dir -= 360.0
        if self.target_dir < 0.0:
            self.target_dir += 360.0

        #push node
        self.count_node += 1
        postnode = PosBag.stk[len(PosBag.stk) - 1]
        PosBag.path[self.count_node] = [postnode, self.position, dir, self.target_dir]
        PosBag.stk.append(self.count_node)
        #print PosBag.path.items()

    def expedition(self):
        drive = Drive_Method()

        if self.set_pose == False and self.range_ahead <= 0.6:
            self.turnCheck(drive)
            drive.publish()
            return

        #print "target dir: ", self.target_dir
        #print "angle: ", self.angle
        #print "pose x: ", self.position_x
        #print "pose y: ", self.position_y
        #print "target pos x:", target_pose[0]
        #print "target pos y:", target_pose[1]
        if self.target_dir <= 10:
            if self.angle < self.target_dir + 5.0:  # match pose
                self.driveLinear(drive)
            elif self.angle < self.target_dir + 10.0:  # match pose
                if self.angle > 180.0:
                    drive.smallTurnLeft()
                else:
                    drive.smallTurnRight()
            else:  # set pose
                if self.angle > 180.0:
                    drive.angleTurnLeft()
                else:
                    drive.angleTurnRight()
        elif self.target_dir >= 350:
            if 355.0 < self.angle:  # match pose
                self.driveLinear(drive)
            elif 350.0 < self.angle:  # match pose
                if self.angle > 180.0:
                    drive.smallTurnLeft()
                else:
                    drive.smallTurnRight()
            else:  # set pose
                if self.angle > 180.0:
                    drive.angleTurnLeft()
                else:
                    drive.angleTurnRight()
        else:
            if self.target_dir - 5.0 < self.angle < self.target_dir + 5.0:  # match pose
                self.driveLinear(drive)
            elif self.target_dir - 10.0 < self.angle < self.target_dir + 10.0:  #match pose
                if self.angle - self.target_dir < 0:
                    drive.smallTurnLeft()
                else:
                    drive.smallTurnRight()
            else:   #set pose
                if self.angle - self.target_dir < 0:
                    drive.angleTurnLeft()
                else:
                    drive.angleTurnRight()
        drive.publish()

    def driveLinear(self, drive):
        self.set_pose = False
        drive.increaseSpeed()
        if self.range_right <= 0.4:
            drive.increaseLeftTurn()
            return
        if self.range_left <= 0.4:
            drive.increaseRightTurn()
            return
        if self.range_right <= 1.0 and self.range_left <= 0:
            if self.range_left > self.range_right:
                drive.increaseLeftTurn()
            else:
                drive.increaseRightTurn()

    def find_target(self):
        for l in list(reversed(PosBag.stk)):
            if 'both' in PosBag.path[l]:
                return l
        return 0

    def escape(self):
        if self.next == 0:
            self.next = PosBag.stk.pop()
        node = PosBag.path[self.next]
        target_pose = node[1]
        drive = Drive_Method()
        target_dir = self.getDirection(target_pose)

        #if self.range_ahead <= 0.5 and self.position_x - 0.2 <= target_pose[0] <= self.position_x + 0.2 and self.position_y - 0.2 <= target_pose[1] <= self.position_y + 0.2:
        if self.position_x - 0.2 <= target_pose[0] <= self.position_x + 0.2 and self.position_y - 0.2 <= target_pose[1] <= self.position_y + 0.2:
            drive.forceStop()
            drive.publish()
            if self.next == self.target_node:
                self.target_dir = 180.0 + PosBag.path[self.target_node][3]
                if self.target_dir >= 360.0:
                    self.target_dir -+ 360.0
                self.next = 0
                self.target_node = 0
                self.set_pose = True
                return
            else:
                self.next = PosBag.stk.pop()
            return #finish

        '''
        print "target dir: ", target_dir
        print "angle: ", self.angle
        print "pose x: ", self.position_x
        print "pose y: ", self.position_y
        print "target pos x:", target_pose[0]
        print "target pos y:", target_pose[1]
        '''

        if target_dir <= 10:
            if self.angle < target_dir + 5.0:  # match pose
                drive.increaseSpeed()
            elif self.angle < target_dir + 10.0:  # match pose
                if self.angle > 180.0:
                    drive.smallTurnLeft()
                else:
                    drive.smallTurnRight()
            else:  # set pose
                if self.angle > 180.0:
                    drive.angleTurnLeft()
                else:
                    drive.angleTurnRight()
        elif target_dir >= 350:
            if 355.0 < self.angle:  # match pose
                drive.increaseSpeed()
            elif 350.0 < self.angle:  # match pose
                if self.angle > 180.0:
                    drive.smallTurnLeft()
                else:
                    drive.smallTurnRight()
            else:  # set pose
                if self.angle > 180.0:
                    drive.angleTurnLeft()
                else:
                    drive.angleTurnRight()
        else:
            if target_dir - 5.0 < self.angle < target_dir + 5.0:  # match pose
                drive.increaseSpeed()
            elif target_dir - 10.0 < self.angle < target_dir + 10.0:  # match pose
                if self.angle - target_dir < 0:
                    drive.smallTurnLeft()
                else:
                    drive.smallTurnRight()
            else:  # set pose
                if self.angle - target_dir < 0:
                    drive.angleTurnLeft()
                else:
                    drive.angleTurnRight()
        drive.publish()

    def getDirection(self, position):
        angle = 180.0 * math.atan2(self.position_y - position[1], self.position_x - position[0]) / math.pi
        if angle < 0:
            angle += 360
        if angle >= 360:
            angle -= 360
        return angle