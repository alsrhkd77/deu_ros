#!/usr/bin/env python
# -*- coding:utf-8 -*-

from sensor_msgs.msg import Image
import rospy, cv2, cv_bridge, numpy
from Ros01team02.msg import rightMomentum

"""
There was a problem about inheriting same parent class. 
That's why I divided Left/Right Image Subscriber.
Only one difference with them is about Publisher part.
They publish different topics from different topic node.
If we need to filter the drive line( it is supposed to be realize in makeOneLine method ),
makeOneLine method will be a different part, too.
"""
class ImageProcessor:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.originImage = None
        rate = rospy.Rate(5)
        self.binarizedImg = None

        self.cx = 0

        self.rightmomentum_sub = rospy.Publisher('rightMomentum',
                                           rightMomentum, queue_size=1)
        self.rightmomentum = rightMomentum()

        print("Right Initiator ImageProcessor")

    def binary(self, msg):
        self.setImage(msg)
        hsv_image = cv2.cvtColor(self.originImage, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_image)
        v = cv2.inRange(v, 200, 225)
        self.binarizedImg = v
        height, width, channel = (self.originImage).shape
        search_top = height / 4 * 3
        search_bot = search_top + 30
        #v[  0:search_top  ,  : ] = 0
        #v[  search_bot:height  ,  :  ] = 0
        v[: , 0:width/2]=0

        cv2.imshow("right binary", v)
        cv2.waitKey(3)
        print("Left binaray Finished")

    def setImage(self,msg):
        self.originImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.originImage, cv2.COLOR_BGR2HSV)

    # make several lines to one line to follow
    def makeOneLine(self):
        pass

    def get_publish_momentum(self):
        M = cv2.moments(self.binarizedImg)
        assert isinstance(self.binarizedImg.shape, object)
        h, w = self.binarizedImg.shape
        if M['m00'] > 0:
            # start 무게중심구하기
            self.cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # end 무게중심구하기
            cv2.circle(self.originImage, (self.cx, cy), 20, (0, 0, 255), -1)

            self.cx = self.cx - 320

            self.rightmomentum.rightMomentum = self.cx
            self.rightmomentum_sub.publish(self.rightmomentum)

            cv2.imshow("right origin", self.originImage)
            cv2.waitKey(3)

            rospy.loginfo(self.cx)
            print("momentum function worked")


class RightImageSubscriber(ImageProcessor):
    def __init__(self):
        ImageProcessor.__init__(self)
        self.image_sub = rospy.Subscriber('my_right_camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        self.binary(msg)
        self.get_publish_momentum()
        print("Right Image callback worked")

if __name__ == '__main__':
    rospy.init_node('right_image_node')
    rightImageSubscriber = RightImageSubscriber()
    print("right end")
    print(RightImageSubscriber.__bases__)
    print("ending")
    rospy.spin()