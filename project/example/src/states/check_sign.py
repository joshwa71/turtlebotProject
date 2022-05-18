#!/usr/bin/env python


from __future__ import division
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from smach import State

from utilities import GoToPose


class CheckSign(State):
    def __init__(self):
        rospy.loginfo('CheckSign state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])
        self.go_to_pose = GoToPose()
        
        self.green_sensitivity = 15
        self.red_sensitivity = 1

        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.linear_motion = Twist()
        self.linear_motion.linear.x = 0
        self.linear_motion.angular.z = 0
        self.flag_red = False
        self.flag_green = False

        # self.theta = 0.0


    def scan(self):

        if (self.flag_green == False) and (self.flag_red == False):
            print('No circle found, adjusting position')

            self.linear_motion.angular.z = 0.5
            self.pub.publish(self.linear_motion)

        # self.flag_red = False
        # self.flag_green = False
        red_contours = None
        green_contours = None
        
        img_msg = rospy.wait_for_message('camera/rgb/image_raw',Image)

        try:
            cv2.startWindowThread()
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)


        hsv_green_lower = np.array([74 -1, 20, 20])
        hsv_green_upper = np.array([75 + 1, 255, 255])
        hsv_red_lower = np.array([0 - self.red_sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.red_sensitivity, 255, 255])

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

        mask_original_with_only_green = cv2.bitwise_and(cv_image,cv_image, mask=mask_green)


        #cv2.imshow('green_mask', mask_green)

        mask_red = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)


        mask_original_with_only_red = cv2.bitwise_and(cv_image,cv_image, mask=mask_red)



        cv2.namedWindow('Camera_Feed_Only_Green')
        cv2.imshow('Camera_Feed_Only_Green', mask_original_with_only_green)
        cv2.namedWindow('Camera_Feed_Only_Red')
        cv2.imshow('Camera_Feed_Only_Red', mask_original_with_only_red)
        cv2.waitKey(1)

        # mask_rg = cv2.bitwise_or(mask_red, mask_green)

        # display_image = cv2.bitwise_and(hsv_image,hsv_image, mask=mask_rg)
            
        # cv2.namedWindow('camera_Feed')
        # cv2.imshow('camera_Feed', display_image)
        # cv2.waitKey(3)

        #red_contours = cv2.findContours(mask_red,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, hierarchy = cv2.findContours(mask_green,mode=cv2.RETR_LIST,method=cv2.RETR_LIST)
        red_contours, hierarchy = cv2.findContours(mask_red,mode=cv2.RETR_LIST,method=cv2.RETR_LIST)

        #green_contours = cv2.findContours(mask_green,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        #rgb_green = cv2.cvtColor(mask_green, cv2.COLOR_HSV2RGB)
        #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # green_grey = cv2.cvtColor(mask_original_with_only_green, cv2.COLOR_BGR2GRAY)
        #red_grey = cv2.cvtColor(mask_red, cv2.COLOR_BGR2GRAY)


        print len(green_contours)


        if len(red_contours) > 0:

            # for i in range (10):
            #     self.linear_motion.angular.z = 0.1745
            #     self.pub.publish(self.linear_motion)

            # Find the index of the largest contour
            for pic, contour in enumerate(red_contours):
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                center, radius = cv2.minEnclosingCircle(contour)
     
                if (radius > 10) and (radius < 100):
                    #print radius

                    # circles = cv2.HoughCircles(gray, cv.CV_HOUGH_GRADIENT, 1, 50)
                    # #print radius
                    # self.circle_detected = False
                    # # ensure at least some circles were found
                    # if circles is not None:
                    #     # convert the (x, y) coordinates and radius of the circles to integers
                    #     circles = np.round(circles[0, :]).astype("int")

                    #     self.circle_detected = True
                    #     print self.circle_detected
                    #     #if (cx > x) and (cx < x + w) and (cy > y) and (cy < y + h):
                    #         #print x, y, w, h
                    if (w>5 and h>5) and (w<2000 and h<2000) and (area > 1200):
                        #cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255,0, 0), 2)
                        self.sign = "Red"
                        print('Red Sign Found')
                        cv2.circle(cv_image, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), 2)
                        cv2.putText(cv_image, "Red Sign", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))
                        self.outcome = 'outcome1'
                        cv2.namedWindow('camera_Feed')
                        cv2.imshow('camera_Feed', cv_image)
                        cv2.waitKey(1)
                        self.itter += 1
                        if self.itter >= 10:
                            self.flag_red = True
                            rospy.set_param('/flag_red', self.flag_red)
                            cv2.imwrite('red_circle.png', cv_image)
                            break

            # self.linear_motion.angular.z = 0.0
            # self.pub.publish(self.linear_motion)

            # print('Capturing image of red circle')

        if len(green_contours) > 0:

            # for i in range (10):
            #     self.linear_motion.angular.z = 0.1745
            #     self.pub.publish(self.linear_motion)

            # Find the index of the largest contour
            for pic, contour in enumerate(green_contours):
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                center, radius = cv2.minEnclosingCircle(contour)
                e_circ_area = 3.14*radius*radius
                ratio = area/e_circ_area
                print radius
     
                if (radius > 10) and (radius < 100) and ratio > 0.6:
                    #print radius
                    #print area

                    # circles = cv2.HoughCircles(green_grey, cv.CV_HOUGH_GRADIENT, 1, 50)
                    # #print radius
                    # self.circle_detected = False
                    # # ensure at least some circles were found


                    if (w>5 and h>5) and (w<2000 and h<2000) and (area > 1200):
                        #cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255,0, 0), 2)
                        self.sign = "Green"
                        print('Green Sign Found')
                        cv2.circle(cv_image, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), 2)
                        cv2.putText(cv_image, "Green Sign", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))
                        self.outcome = 'outcome1'
                        cv2.namedWindow('camera_Feed')
                        cv2.imshow('camera_Feed', cv_image)
                        cv2.waitKey(1)
                        self.itter += 1
                        if self.itter >= 10:
                            self.flag_green = True
                            rospy.set_param('/flag_green', self.flag_green)
                            cv2.imwrite('green_circle.png', cv_image)
                            break

            # self.linear_motion.angular.z = 0.0
            # self.pub.publish(self.linear_motion)

            # print('Capturing image of green circle')

        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', cv_image)
        cv2.waitKey(1)



        # if self.theta >= 6.3: # Rotate 360 degrees
        #     self.theta = 0




            # # Getting the parameter for current approach position
            # current_approach_position = rospy.get_param("/current_approach_position")

            # # saving pos and orientation data to pass to go_to_pose class
            # x = current_approach_position[0]
            # y = current_approach_position[1]
            # self.theta += 0.25
            # position = {'x': x, 'y' : y}
            # quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(self.theta/2.0), 'r4' : np.cos(self.theta/2.0)}
            
            # # Sending command to turtlebot to go to requested position
            # self.go_to_pose.goto(position, quaternion)

        
        # print len(green_contours)
        # print len(red_contours)



    def execute(self, userdata, wait=True):
        rospy.loginfo('CheckSign state executing')
        self.itter = 0

        self.flag_red = False
        self.flag_green = False
        
        self.bridge = CvBridge()

        while ((self.flag_green == False) and (self.flag_red == False)):
            self.scan()

        current_location = rospy.get_param("/location")
        flag_green = rospy.get_param("/flag_green")
        flag_red = rospy.get_param("/flag_red")
        if current_location == 'room1_entrance':
            if ((flag_green == False) or (flag_red == False)):
                next_location = 'room2_entrance'
            elif ((flag_green == True) and (self.flag_red == True) and (flag_red == True)):
                next_location = 'room2_centre'
            elif ((flag_green == True) and (self.flag_green == True) and (flag_red == True)):
                next_location = 'room1_centre'
        elif current_location == 'room2_entrance':
            if ((flag_green == False) or (flag_red == False)):
                next_location = 'room1_entrance'
            elif ((flag_green == True) and (self.flag_red == True) and (flag_red == True)):
                next_location = 'room1_centre'
            elif ((flag_green == True) and (self.flag_green == True) and (flag_red == True)):
                next_location = 'room2_centre'
        
        rospy.set_param('/next_location', next_location)



        # # Ensure that the node continues running with rospy.spin()
        # # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
        # try:
        #     rospy.spin()
        # except KeyboardInterrupt:
        #     print("Shutting down")
        #Check if the node is executing in the main path
        # Remember to destroy all image windows before closing node
        
        cv2.destroyAllWindows() 

        return self.outcome







    
