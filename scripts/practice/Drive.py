#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Drive:
    def __init__(self):
        self.speed = 0
        self.turn = 0
        rospy.init_node('keys_to_twist')
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


    def publish(self):
        t = Twist()
        t.angular.z = self.turn
        t.linear.x = self.speed
        self.twist_pub.publish(t)


class Drive_Keys(Drive):
    def __init__(self):
        self.key_mapping = {'w': [0, 1], 'x': [0, 0], 'q': [-1, 0], 'e': [1, 0], 's': [0, -1], 'a': [-1, 1],
                            'd': [1, 1]}
        rospy.Subscriber('keys', String, self.keys_cb, self.twist_pub)
        rospy.spin()


    def keys_cb(self, msg, twist_pub):
        if len(msg.data) == 0 or not self.key_mapping.has_key(msg.data[0]):
            return  # Unknown Key
        vels = self.key_mapping[msg.data[0]]
        self.turn = vels[0]
        if msg.data[0] != 'a' or msg.data != 'd':
            self.speed = vels[1]
        self.publish()

#터틀은 하나인데 turn이랑 speed를 공유해야함 ->싱글턴 적용할 것
class Drive_Method(Drive):
    def __init__(self):
        rospy.Subscriber('keys', String, self.keys_cb, self.twist_pub)
        rospy.spin()

if __name__ == "__main__":
    drive = Drive()