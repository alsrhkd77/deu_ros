#! /usr/bin/env python

from Drive import Drive, Drive_Method
import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Laser_Scan:
    def __init__(self):
        self.range_ahead = 0.0
        self.range_left = 0.0
        self.range_right = 0.0
        self.left = 0.0
        self.right = 0.0
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        #rospy.spin()

    def scan_callback(self, msg):
        #get right angle ranges
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]  # 180
        self.range_right = msg.ranges[len(msg.ranges) / 4]      # 90
        self.range_left = msg.ranges[len(msg.ranges) * 3 / 4]  # 270

        #get nearest range
        #2.0 > right or left
        self.right = reduce(lambda x, y: x if x > y else y, msg.ranges[len(msg.ranges) / 4:len(msg.ranges) / 2])
        self.left = reduce(lambda x, y: x if x > y else y, msg.ranges[len(msg.ranges) / 2:len(msg.ranges) * 3 / 4])

    def print_laser_status(self):
        print "range ahead: %0.1f" % self.range_ahead
        print "range left: %0.1f" % self.range_left
        print "range right: %0.1f" % self.range_right
        print "left: %0.1f" % self.left
        print "right: %0.1f" % self.right


class Pose_Scan:
    def __init__(self):
        self.angle = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.position = [0, 0]
        self.pose_sub = rospy.Subscriber('odom', Odometry, self.pose_callback)

    def pose_callback(self, msg):
        ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler = euler_from_quaternion(ori)
        self.angle = round((euler[2] * 180.0 / math.pi) + 180.0, 2) #use yaw, +-15.0
        self.position_x = round(msg.pose.pose.position.x, 2)
        self.position_y = round(msg.pose.pose.position.y, 2)
        self.position = [self.position_x, self.position_y]
        #self.print_pose_status()

    def print_pose_status(self):
        print "angle: %0.1f" % self.angle
        print "pos_x: %0.1f" % self.position_x
        print "pos_y: %0.1f" % self.position_y

        '''
        print "roll: %0.1f" % euler[0]
        print "pitch: %0.1f" % euler[1]
        print "yaw: %0.1f" % euler[2]
        '''


class Scan(Laser_Scan, Pose_Scan):
    def __init__(self):
        Laser_Scan.__init__(self)
        Pose_Scan.__init__(self)


'''
Must Erase at last
if __name__ == "__main__":
    rospy.init_node('scanner')
    drive = Drive()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        drive.publish()
        rate.sleep()
'''