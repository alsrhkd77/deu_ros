#! /usr/bin/env python

import Drive_vel
from Keys import Keys

import sys, select, tty, termios

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Drive:
    def __init__(self):
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=8)


    def publish(self):
        if Drive_vel.turn > 0:
            Drive_vel.turn -= 0.1
            if Drive_vel.turn < 0.2:
                Drive_vel.turn = 0
        elif Drive_vel.turn < 0:
            Drive_vel.turn += 0.1
            if Drive_vel.turn > -0.2:
                Drive_vel.turn = 0
        t = Twist()
        t.angular.z = Drive_vel.turn
        t.linear.x = Drive_vel.speed
        self.twist_pub.publish(t)
        print "publish speed: %f, turn %f", Drive_vel.speed, Drive_vel.turn


class Drive_Keys(Drive):
    def __init__(self):
        Drive.__init__(self)
        self.key_mapping = {'w': [0, 1], 'x': [0, 0], 'q': [1, 0], 'e': [-1, 0], 's': [0, -1], 'a': [1, 1],
                            'd': [-1, 1]}
        rospy.Subscriber('keys', String, self.keys_cb, self.twist_pub)
        #rospy.spin()


    def keys_cb(self, msg, twist_pub):
        if len(msg.data) == 0 or not self.key_mapping.has_key(msg.data[0]):
            return  # Unknown Key
        vels = self.key_mapping[msg.data[0]]
        Drive_vel.turn = vels[0]
        if msg.data[0] != 'a' or msg.data != 'd':
            Drive_vel.speed = vels[1]
        self.publish()



class Drive_Method(Drive):
    def __init__(self):
        Drive.__init__(self)
        #rospy.Subscriber('keys', String, self.keys_cb, self.twist_pub)
        #rospy.spin()

    def increaseSpeed(self):
        Drive_vel.speed += 0.1
        if Drive_vel.speed > 1:
            Drive_vel.speed = 1
        self.publish()


    def decreaseSpeed(self):
        Drive_vel.speed -= 0.1
        if (Drive_vel.speed < -1):
            Drive_vel.speed = -1
        self.publish()


    def increaseLeftTurn(self):
        Drive_vel.turn += 0.2
        if(Drive_vel.turn > 2):
            Drive_vel.turn = 2
        self.publish()


    def increaseRightTurn(self):
        Drive_vel.turn -= 0.2
        if(Drive_vel.turn < -2):
            Drive_vel.turn = -2
        self.publish()


    def forceStop(self):
        Drive_vel.speed = 0
        Drive_vel.turn = 0
        print "stop"
        self.publish()


    def turnLeft(self):
        Drive_vel.speed = 0
        Drive_vel.turn = 2
        print "left"
        self.publish()


    def turnRight(self):
        Drive_vel.speed = 0
        Drive_vel.turn = -2
        print "right"
        self.publish()


if __name__=="__main__":
    rospy.init_node('drive_car')
    drive = Drive()
    drive_keys = Drive_Keys()
    keys = Keys()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        keys.publish()
        drive.publish()
        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, keys.old_attr)