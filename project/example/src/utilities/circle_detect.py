#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class circleIdentifier():
    def __init__(self):
        self.sensitivity = 2
        self.movement = Twist()
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self.linear_motion = Twist()
        self.linear_motion.linear.x = 0
        self.linear_motion.angular.z = 0
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.flag_red = False
        self.flag_green = False

    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

        mask_red = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        # mask_rg = cv2.bitwise_or(mask_red, mask_green)

        # display_image = cv2.bitwise_and(hsv_image,hsv_image, mask=mask_rg)
            
        # cv2.namedWindow('camera_Feed')
        # cv2.imshow('camera_Feed', display_image)
        # cv2.waitKey(3)

        #red_contours = cv2.findContours(mask_red,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, hierarchy = cv2.findContours(mask_green,mode=cv2.RETR_LIST,method=cv2.RETR_LIST)
        red_contours, hierarchy = cv2.findContours(mask_red,mode=cv2.RETR_LIST,method=cv2.RETR_LIST)

        #green_contours = cv2.findContours(mask_green,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        

        if len(red_contours) == 0 and len(green_contours) == 0:
            print('No circle found, adjusting position')
            # for i in range (30):
            #     self.linear_motion.angular.z = 0.1745
            #     self.pub.publish(self.linear_motion)

            self.linear_motion.angular.z = 0.2
            rospy.sleep(0.1) # Sleeps for 1 sec to ensure full view of circle
            self.pub.publish(self.linear_motion)
        
        # print len(green_contours)
        # print len(red_contours)

        if len(red_contours) > 0:
            self.flag_red = True

            for i in range (10):
                self.linear_motion.angular.z = 0.1745
                self.pub.publish(self.linear_motion)

            # Find the index of the largest contour
            for pic, contour in enumerate(red_contours):
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                center, radius = cv2.minEnclosingCircle(contour)
                #print radius
                if (radius > 20) and (radius < 50):
                    #print x, y, w, h
                    if (w>5 and h>5) and (w<2000 and h<2000):
                        #cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255,0, 0), 2)
                        self.sign = "Red"
                        print('Red Sign Found')
                        cv2.circle(cv_image, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), 2)
                        cv2.putText(cv_image, "Red Sign", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))

            self.linear_motion.angular.z = 0
            self.pub.publish(self.linear_motion)

            print('Capturing image of red circle')
            cv2.imwrite('red_circle.png', cv_image)

        if len(green_contours) > 0:
            self.flag_green = True

            for i in range (10):
                self.linear_motion.angular.z = 0.1745
                self.pub.publish(self.linear_motion)

            # Find the index of the largest contour
            for pic, contour in enumerate(green_contours):
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                center, radius = cv2.minEnclosingCircle(contour)
                #print radius
                if (radius > 20) and (radius < 50):
                    if (w>5 and h>5) and (w<2000 and h<2000):
                        #cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255,0, 0), 2)
                        self.sign = "Green"
                        print('Green Sign Found')
                        cv2.circle(cv_image, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), 2)
                        cv2.putText(cv_image, "Green Sign", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))

            self.linear_motion.angular.z = 0
            self.pub.publish(self.linear_motion)

            print('Capturing image of green circle')
            cv2.imwrite('green_circle.png', cv_image)
            



        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', cv_image)

def main(args):
    rospy.init_node('circle_identifier', anonymous=True)
    cI = circleIdentifier()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)






    
