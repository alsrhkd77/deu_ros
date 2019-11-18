#! /usr/bin/env python

from Drive import Drive, Drive_Method
import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Scan:
    def __init__(self):
        self.range_ahead = 0.0;
        self.range_left = 0.0;
        self.range_right = 0.0;
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        #rospy.spin()

    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]  # 180
        self.range_right = msg.ranges[len(msg.ranges) / 4]      # 90
        self.range_left = msg.ranges[len(msg.ranges) * 3 / 4]  # 270

        #right = reduce(lambda x, y: x if x > y else y, msg.ranges[45:135])
        #2.0 > right or left
        right = reduce(lambda x, y: x if x > y else y, msg.ranges[len(msg.ranges) / 8:len(msg.ranges) * 3 / 8])
        left = reduce(lambda x, y: x if x > y else y, msg.ranges[len(msg.ranges) * 5 / 8:len(msg.ranges) * 7 / 8])

        '''
        print "range ahead: %0.1f" % self.range_ahead
        print "range left: %0.1f" % self.range_left
        print "range right: %0.1f" % self.range_right
        print "left: %0.1f" % left
        print "right: %0.1f" % right
        '''
        #drive = Drive()
        drive = Drive_Method()
        if self.range_ahead <= 0.4:
            drive.forceStop()
            #Drive_vel.forceStop()
        elif self.range_left <= 0.4:
            pass
            #drive.turnRight()
            #Drive_vel.turnRight()
        elif self.range_right <= 0.4:
            pass
            #drive.turnLeft()
            #Drive_vel.turnLeft()
        else:
            pass
            #drive.increaseSpeed()
            #Drive_vel.increaseSpeed()
        drive.publish()

class Pose_scan:
    def __init__(self):
        self.pose_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.pose_callback)
        self.position_x = 0
        self.position_y = 0

    def pose_callback(self, msg):
        ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler = euler_from_quaternion(ori)
        pose =  euler[2] #use yaw
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        print "angle: %0.1f" % pose
        print "pos_x: %0.1f" % self.position_x
        print "pos_y: %0.1f" % self.position_y

        '''
        print "roll: %0.1f" % euler[0]
        print "pitch: %0.1f" % euler[1]
        print "yaw: %0.1f" % euler[2]
        '''

'''
if __name__ == "__main__":
    rospy.init_node('scanner')
    drive = Drive()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        drive.publish()
        rate.sleep()
'''