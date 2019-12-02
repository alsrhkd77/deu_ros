#! /usr/bin/env python

import rospy
from lane_follower import Lane_follower
from blockingbar_detector import BlockingBar_Detector
from stop_line_detector import Stop_line_detactor
from flover import Flover
from obstacle_detecter import Obstacle_detecter

if __name__ == "__main__":
    rospy.init_node('car_test')
    follower = Lane_follower()
    #bar_detector = BlockingBar_Detector()
    #stop_line = Stop_line_detactor()
    #turn = Flover()
    #obstacle = Obstacle_detecter()
    rospy.spin()