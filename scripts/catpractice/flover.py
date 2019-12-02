#! /usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from drive import Drive_Method

class Flover:
    def __init__(self):
        self.angle = 0.0
        #self.position_x = 0.0
        #self.position_y = 0.0
        #self.position = [0, 0]
        self.pose_sub = rospy.Subscriber('odom', Odometry, self.pose_callback)
        self.set_pose = True
        self.target_dir = 180

    def pose_callback(self, msg):
        ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler = euler_from_quaternion(ori)
        self.angle = round((euler[2] * 180.0 / math.pi) + 180.0, 2) #use yaw, +-15.0
        if self.set_pose:
            drive = Drive_Method()
            turn_angle = abs(self.target_dir - self.angle)
            if self.target_dir - 2.0 < self.angle < self.target_dir + 2.0:
                self.set_pose = False
                return  # finish set
            else:
                drive.forceAngle(turn_angle)
            drive.publish()
        '''
        self.position_x = round(msg.pose.pose.position.x, 2)
        self.position_y = round(msg.pose.pose.position.y, 2)
        self.position = [self.position_x, self.position_y]
        '''
        self.print_pose_status()

    def print_pose_status(self):
        print "angle: %0.1f" % self.angle
        #print "pos_x: %0.1f" % self.position_x
        #print "pos_y: %0.1f" % self.position_y