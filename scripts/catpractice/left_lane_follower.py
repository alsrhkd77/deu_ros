#! /usr/bin/env python
import math

import rospy
import cv2
from scan_image import Scan_image
from drive import Drive_Method


class Left_lane_follower(Scan_image):
    def __init__(self):
        Scan_image.__init__(self, 'left')

    def image_callback(self, msg):
        Scan_image.image_callback(self, msg)
        drive = Drive_Method()
        err = self.cx - 45 - (640 / 2)
        err = -float(err) / 100
        print err
        drive.setTurn(err)
        drive.straight()
        drive.publish()

        self.view() # erase at last

self.image[0:j , 2:4]

    def view(self):
        cv2.circle(self.image, (self.cx, self.cy), 20, (0, 0, 255), -1)
        cv2.imshow("left", self.image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('test')
    follower = Left_lane_follower()
    rospy.spin()

