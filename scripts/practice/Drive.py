#! /usr/bin/env python

import Drive_vel

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

'''
Common used Twist publisher class
'''
class Drive:
    def __init__(self):
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def publish(self):
        #Publish
        t = Twist()
        t.angular.z = Drive_vel.turn
        t.linear.x = Drive_vel.speed
        self.twist_pub.publish(t)
        #print "publish speed: %f, turn %f", Drive_vel.speed, Drive_vel.turn
        # decrease turn
        if Drive_vel.turn > 0:
            Drive_vel.turn -= 0.1
            if Drive_vel.turn < 0.2:
                Drive_vel.turn = 0
        elif Drive_vel.turn < 0:
            Drive_vel.turn += 0.1
            if Drive_vel.turn > -0.2:
                Drive_vel.turn = 0


'''
Publish Twist MSG use key input
w: go
s: back
a: increse turn left
d: increse turn right
q: turn left
e: turn right
x: force stop
'''
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


'''
Publish Twist MSG use Method
'''
class Drive_Method(Drive):
    def __init__(self):
        Drive.__init__(self)
        # rospy.Subscriber('keys', String, self.keys_cb, self.twist_pub)
        # rospy.spin()

    def increaseSpeed(self):
        Drive_vel.speed += 0.1
        if Drive_vel.speed > 1:
            Drive_vel.speed = 1

    def decreaseSpeed(self):
        Drive_vel.speed -= 0.1
        if (Drive_vel.speed < -1):
            Drive_vel.speed = -1

    def increaseLeftTurn(self):
        Drive_vel.turn += 0.4
        if (Drive_vel.turn > 2):
            Drive_vel.turn = 2

    def increaseRightTurn(self):
        Drive_vel.turn -= 0.4
        if (Drive_vel.turn < -2):
            Drive_vel.turn = -2

    def forceStop(self):
        Drive_vel.speed = 0
        Drive_vel.turn = 0

    def turnLeft(self):
        Drive_vel.speed = 0
        Drive_vel.turn = 2

    def turnRight(self):
        Drive_vel.speed = 0
        Drive_vel.turn = -2

    def fullSpeed(self):
        Drive_vel.speed = 1.0
        Drive_vel.turn = 0

    def smallTurnLeft(self):
        Drive_vel.speed = 0
        Drive_vel.turn = 0.4

    def smallTurnRight(self):
        Drive_vel.speed = 0
        Drive_vel.turn = -0.4

    def angleTurnLeft(self):
        Drive_vel.speed = 0
        Drive_vel.turn = 0.1

    def angleTurnRight(self):
        Drive_vel.speed = 0
        Drive_vel.turn = -0.1


'''
if __name__ == "__main__":
    rospy.init_node('drive_car')
    drive = Drive()
    drive_keys = Drive_Keys()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        drive.publish()
        rate.sleep()
'''
