#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from Ros01team02.msg import leftMomentum
from Ros01team02.msg import rightMomentum
from geometry_msgs.msg import Twist
import message_filters

class Drive:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        # movement value
        self.twist = Twist()
        self.foward_value = 0
        self.angular_value = 0

    def set_foward_value(self, value):
        self.foward_value = value

    def set_angular_value(self, value):
        self.angular_value = value

    def move(self):
        self.cmd_vel_pub.publish(self.twist)


"""
topic이 두 개 동시에 들어온다면 combo_callback,
1개면 각각의 right_callback, left_callback이 실행되도록 하면 좋을 것 같은데.
ㅋ
뮤텍스락같은게되겟나..ㅎ_ㅎ

Drive 클래스 상속고치다가 커밋함.
"""
class NormalDrive(Drive):
    def __init__(self):
        Drive.__init__(self)
        # when listen 2 topics
        self.rightLine_sub_combine = message_filters.Subscriber('rightMomentum',rightMomentum)
        self.leftLine_sub_combine = message_filters.Subscriber('leftMomentum', leftMomentum)
        ts = message_filters.ApproximateTimeSynchronizer([self.leftLine_sub_combine, self.rightLine_sub_combine], 10,
                                                         0.1, allow_headerless=True)
        ts.registerCallback(self.momentum_callback)

        # when listen only one topic
        self.rightLine_sub = rospy.Subscriber('rightMomentum', rightMomentum, self.right_callback)
        self.leftLine_sub = rospy.Subscriber('leftMomentum', leftMomentum, self.right_callback)

        print("End Initiator NormalDrive")

# BEGIN PREPARE FOR SUBSCRIBING 2 TOPICS
    #미 친 드디 어 ㅠ싱크로나이즈하 게토픽받는거해  따개기 쁨
    """
    토픽 2개 싱크로나이즈하는거 성공했으나 class형으로 파일형이나와서 강제로 float으로 만들어주기 위해 발악 함^_^ 
    """
    def momentum_callback(self, left, right):
        flt_left = self.casting_float(left)
        flt_right = self.casting_float(right)

        err = (flt_left + flt_right) / 2

        self.twist.linear.x = 0.7
        self.twist.angular.z = -float(err) / 1000

        print(" left : " + str(flt_left) + " right : " + str(flt_right) )
        print("err : " + str(self.twist.angular.z))

        self.cmd_vel_pub.publish(self.twist)
        print("End combo Callback")

    """
    토픽 두개 싱크로나이즈하는거 성공했으나 class형으로 파일형이나와서 강제로 float으로 만들어주기 위해 발악 함^_^ 
    타입캐스팅 함수
    """
    def casting_float(self, val):
        mid_str = str(val)
        result = mid_str.split(" ", 1)
        return float(result[1])
# END PREPARE FOR SUBSCRIBING 2 TOPICS

# RIGHT TOPIC CALLBACK
    def right_callback(self, topic):
        self.twist.linear.x = 1.0
        self.twist.angular.z = float(-(topic / 2) / 1000)
        print(" right : " + str(self.twist.angular.z) )
        self.cmd_vel_pub.publish(self.twist)
        print("End right Callback")

#LEFT TOPIC CALLBACK
    def left_callback(self, topic):
        self.twist.linear.x = 1.0
        self.twist.angular.z = float(-(topic / 2) / 1000)
        print(" left : " + str(self.twist.angular.z) )
        self.cmd_vel_pub.publish(self.twist)
        print("End left Callback")





if __name__ == '__main__':
    rospy.init_node('Normal_driving_mode')
    normal = NormalDrive()
    rospy.spin()
