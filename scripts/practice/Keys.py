#! /usr/bin/env python

import sys, select, tty, termios
import rospy
from std_msgs.msg import String

'''
w: go
s: back
a: increse turn left
d: increse turn right
q: turn left
e: turn right
x: force stop
'''

class Keys:
    def __init__(self):
        self.key_pub = rospy.Publisher('keys', String, queue_size=1)
        #rospy.init_node("keyboard_driver")
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print "Publishing keystrokes. Press Ctrl-C to exit.."

    def publish(self):
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            self.key_pub.publish(sys.stdin.read(1))


'''
if __name__ == "__main__":
    keys = Keys()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        keys.publish()
        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, keys.old_attr)
'''