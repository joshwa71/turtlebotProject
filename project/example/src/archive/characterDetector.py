#!/usr/bin/env python
# This second piece of skeleton code will be centred around
# combining colours and creating a mask

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class Identifier():
	def __init__(self):
        
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
		self.sensitivity = 5
	def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
			
			
		hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
		gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
		
		cascPath = "haar.xml"
		face = cv2.CascadeClassifier(cascPath)	
		
			
		#RED FOR SCARLET
		hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
		hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
		
		red_mask = cv2.inRange(hsv_img, hsv_red_lower, hsv_red_upper)
		red_contours, hierarchy = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		# Find the index of the largest contour
		for pic, contour in enumerate(red_contours):
			area = cv2.contourArea(contour)
			if(area > 1000):
				x, y, w, h = cv2.boundingRect(contour)
				if w>80 and h>100:
					cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
					cv2.putText(cv_img, "scarlet", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
           			
					faces = face.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.cv.CV_HAAR_SCALE_IMAGE )
		
					for (xx, yy, ww, hh) in faces:
						cv2.rectangle(cv_img, (xx, yy), (xx+ww, yy+hh), (0, 255, 0), 2)
					rospy.loginfo("Scarlet Detected")
           			

		
		#BLUE FOR PEACOCK
		hsv_blue_lower = np.array([210 - self.sensitivity, 100, 100])
		hsv_blue_upper = np.array([210 + self.sensitivity, 255, 255])
		
		blue_mask = cv2.inRange(hsv_img, hsv_blue_lower, hsv_blue_upper)
		blue_contours, hierarchy = cv2.findContours(blue_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		# Find the index of the largest contour
		for pic, contour in enumerate(blue_contours):
			area = cv2.contourArea(contour)
			if(area > 1000):
				x, y, w, h = cv2.boundingRect(contour)
				if w>80 and h>100:
					cv2.rectangle(cv_img, (x, y), (x + w, y + h), (255,0, 0), 2)
           			cv2.putText(cv_img, "peacock", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))
					faces = face.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.cv.CV_HAAR_SCALE_IMAGE )
		
					for (xx, yy, ww, hh) in faces:
						cv2.rectangle(cv_img, (xx, yy), (xx+ww, yy+hh), (0, 255, 0), 2)
					rospy.loginfo("Peacock Detected")
       
           			
       	#BLUE FOR MUSTARD
		hsv_ylw_lower = np.array([52 - self.sensitivity, 100, 100])
		hsv_ylw_upper = np.array([52 + self.sensitivity, 255, 255])
		
		ylw_mask = cv2.inRange(hsv_img, hsv_ylw_lower, hsv_ylw_upper)
		ylw_contours, hierarchy = cv2.findContours(ylw_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		# Find the index of the largest contour
		for pic, contour in enumerate(ylw_contours):
			area = cv2.contourArea(contour)
			if(area > 1000):
				x, y, w, h = cv2.boundingRect(contour)
				if w>80 and h>100:
					cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0,255, 255), 2)
           			cv2.putText(cv_img, "mustard", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

					faces = face.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.cv.CV_HAAR_SCALE_IMAGE )
		
					for (xx, yy, ww, hh) in faces:
						cv2.rectangle(cv_img, (xx, yy), (xx+ww, yy+hh), (0, 255, 0), 2)
					rospy.loginfo("Mustard Detected")

		#PURPLE FOR PLUM
		hsv_prple_lower = np.array([52 - self.sensitivity, 100, 100])
		hsv_prple_upper = np.array([52 + self.sensitivity, 255, 255])
		
		prple_mask = cv2.inRange(hsv_img, hsv_prple_lower, hsv_prple_upper)
		prple_contours, hierarchy = cv2.findContours(prple_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		# Find the index of the largest contour
		for pic, contour in enumerate(prple_contours):
			area = cv2.contourArea(contour)
			if(area > 1000):
				x, y, w, h = cv2.boundingRect(contour)
				if w>80 and h>100:
					cv2.rectangle(cv_img, (x, y), (x + w, y + h), (108,0, 108), 2)
           			cv2.putText(cv_img, "plum", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (108,0,108))

					faces = face.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.cv.CV_HAAR_SCALE_IMAGE )
		
					for (xx, yy, ww, hh) in faces:
						cv2.rectangle(cv_img, (xx, yy), (xx+ww, yy+hh), (0, 255, 0), 2)
					rospy.loginfo("Plum Detected")
		
		
		cv2.namedWindow('camera_Feed')
		cv2.imshow('camera_Feed', cv_img)
		cv2.waitKey(3)
		
		
		
def main(args):
 # Instantiate your class
    # And rospy.init the entire node
	cI = Identifier()
	rospy.init_node('step', anonymous=True)
    
   	# Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
    # Remember to destroy all image windows before closing node
	cv2.destroyAllWindows() 
	#Check if the node is executing in the main path
	
if __name__ == '__main__':
    main(sys.argv)		
		
		
		

		
		
		
		
		
		
		
		
		
