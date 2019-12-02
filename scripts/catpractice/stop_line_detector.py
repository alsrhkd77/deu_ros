#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from scan_image import Scan_image
from drive import Drive_Method

class Stop_line_detactor(Scan_image):
    def __init__(self):
        Scan_image.__init__(self, 'center', 0)
        self.stop_pub = rospy.Publisher('stop_sign', String, queue_size=1)

    def image_callback(self, msg):
        Scan_image.image_callback(self, msg)
        drive = Drive_Method()
        '''
        h, w = self.mask.shape
        self.mask[0:h * 3 / 5, 0:w] = 0
        self.mask[h - (h / 8):h, 0:w] = 0
        self.mask[0:h, 0:w / 4] = 0
        self.mask[0:h, w - (w / 4):w] = 0
        '''

        #mask = cv2.Canny(self.mask, 100, 200)

        ret, thr = cv2.threshold(self.mask, 0, 255, 0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) <= 0:
            return  #not found

        '''
        x,y,w,h = cv2.boundingRect(cnt)
        self.mask = cv2.rectangle(self.mask,(x,y),(x+w,y+h),(0,0,255),2)
        cv2.drawContours(self.mask, [cnt], 0, (255, 255, 0), 1)
        '''

        font = cv2.FONT_HERSHEY_SIMPLEX  # normal size sans-serif font
        fontScale = 1.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            epsilon1 = 0.1 * cv2.arcLength(cnt, True)
            approx1 = cv2.approxPolyDP(cnt, epsilon1, True)
            cv2.drawContours(self.mask, [approx1], 0, (0, 255, 0), 3)
            x, y, w, h = cv2.boundingRect(cnt)
            if 11000.0 < area < 13000.0:  # need to find area's max range
                msg = String()
                msg.data = "stop"
                #print "stop"
                drive.stop_sign()
                self.stop_pub.publish(msg)
                # rospy.sleep(5)
                cv2.putText(self.mask, str(area), (x, y), font, fontScale, (255, 0, 255), 2)
            else:
                drive.go_sign()

        #cv2.imshow("window", self.image)
        #cv2.imshow("window", hsv)
        #cv2.imshow("window1", self.mask)
        cv2.waitKey(3)