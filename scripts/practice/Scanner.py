#! /usr/bin/env python


from Drive import Drive_Method
import Drive_vel
import rospy
from sensor_msgs.msg import LaserScan


class Scanner:
    def __init__(self):
        self.drive = Drive_Method()
        self.range_ahead = 0.0;
        self.range_left = 0.0;
        self.range_right = 0.0;
        rospy.init_node('range_ahead')
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        rospy.spin()


    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]
        self.range_left = msg.ranges[len(msg.ranges) / 4]
        self.range_right = msg.ranges[len(msg.ranges) * 3 / 4]
        #angle_min_in_degree = msg.angle_min * 180.0 / math.pi
        #angle_max_in_degree = msg.angle_max * 180.0 / math.pi
        #angle_inc_in_degree = msg.angle_increment * 180.0 / math.pi
        #rospy.loginfo('angle_min = %f, angle_max = %f, angle_inc = %f', angle_min_in_degree, angle_max_in_degree, angle_inc_in_degree)
        print "range ahead: %0.1f" % self.range_ahead
        print "range left: %0.1f" % self.range_left
        print "range right: %0.1f" % self.range_right
        #0.5
        if self.range_ahead <= 0.5:
            Drive_vel.speed = 0
            Drive_vel.turn = 0
            #self.drive.forceStop()
            self.drive.publish()
        elif self.range_left <= 0.5:
            #self.drive.turnRight()
            Drive_vel.turn = -1
            self.drive.publish()
        elif self.range_right <= 0.5:
            #self.drive.turnLeft()
            Drive_vel.turn = 1
            self.drive.publish()


if __name__ == "__main__":
    scanner = Scanner()