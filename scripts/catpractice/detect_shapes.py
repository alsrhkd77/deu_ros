#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32

class Detect_shapes:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.left = None
        self.right = None
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.left_sub = rospy.Subscriber('my_left_camera/rgb/image_raw', Image, self.left_callback)
        self.right_sub = rospy.Subscriber('my_right_camera/rgb/image_raw', Image, self.right_callback)


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        _, _, img = cv2.split(img)
        #img = cv2.inRange(img, 200, 225)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        #mask = cv2.inRange(hsv, lower_white, upper_white)

        '''
        h, w = mask.shape
        
        mask[0:h * 3 / 5, 0:w] = 0
        mask[h - (h / 8):h, 0:w] = 0
        mask[0:h, 0:w / 4] = 0
        mask[0:h, w - (w / 4):w] = 0
        '''

        #ret, thr = cv2.threshold(img, 127, 255, 0)
        ret, thr = cv2.threshold(img, 200, 255, 0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) <= 0:
            return  #not found

        #cnt = contours[0]

        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
        cv2.drawContours(img, contours, -1, (255, 255, 0), 1)
        '''
        epsilon1 = 0.1*cv2.arcLength(cnt, True)
        approx1 = cv2.approxPolyDP(cnt, epsilon1, True)

        cv2.drawContours(image, [approx1], 0, (0, 255, 0), 3)
        '''
        #cv2.imshow("window", image)
        #cv2.imshow("window", hsv)
        cv2.imshow("center", img)
        cv2.imshow("left", self.left)
        cv2.imshow("right", self.right)
        cv2.waitKey(3)

    def left_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        _, _, img = cv2.split(img)
        # img = cv2.inRange(img, 200, 225)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        # mask = cv2.inRange(hsv, lower_white, upper_white)

        '''
        h, w = mask.shape

        mask[0:h * 3 / 5, 0:w] = 0
        mask[h - (h / 8):h, 0:w] = 0
        mask[0:h, 0:w / 4] = 0
        mask[0:h, w - (w / 4):w] = 0
        '''

        # ret, thr = cv2.threshold(img, 127, 255, 0)
        ret, thr = cv2.threshold(img, 200, 255, 0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) <= 0:
            return  # not found

        # cnt = contours[0]

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.drawContours(img, contours, -1, (255, 255, 0), 1)
        '''
        epsilon1 = 0.1*cv2.arcLength(cnt, True)
        approx1 = cv2.approxPolyDP(cnt, epsilon1, True)

        cv2.drawContours(image, [approx1], 0, (0, 255, 0), 3)
        '''
        # cv2.imshow("window", image)
        # cv2.imshow("window", hsv)
        self.left = img
        #cv2.imshow("left", img)
        #cv2.waitKey(3)

    def right_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        _, _, img = cv2.split(img)
        # img = cv2.inRange(img, 200, 225)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        # mask = cv2.inRange(hsv, lower_white, upper_white)

        '''
        h, w = mask.shape

        mask[0:h * 3 / 5, 0:w] = 0
        mask[h - (h / 8):h, 0:w] = 0
        mask[0:h, 0:w / 4] = 0
        mask[0:h, w - (w / 4):w] = 0
        '''

        # ret, thr = cv2.threshold(img, 127, 255, 0)
        ret, thr = cv2.threshold(img, 200, 255, 0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) <= 0:
            return  # not found

        # cnt = contours[0]

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.drawContours(img, contours, -1, (255, 255, 0), 1)
        '''
        epsilon1 = 0.1*cv2.arcLength(cnt, True)
        approx1 = cv2.approxPolyDP(cnt, epsilon1, True)

        cv2.drawContours(image, [approx1], 0, (0, 255, 0), 3)
        '''
        # cv2.imshow("window", image)
        # cv2.imshow("window", hsv)
        self.right = img
        #cv2.imshow("right", img)
        #cv2.waitKey(3)

if __name__ == "__main__":
    rospy.init_node('shape_finder')
    center = Detect_shapes()
    '''
    center = Detect_shapes('camera/rgb/image_raw')
    left = Detect_shapes('my_left_camera/rgb/image_raw')
    right = Detect_shapes('my_right_camera/rgb/image_raw')
    '''
    rospy.spin()
    # END ALL