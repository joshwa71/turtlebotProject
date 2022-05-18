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
			
			
		cascPath = "haar.xml"
		face = cv2.CascadeClassifier(cascPath)	
		
		
		gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
		
		faces = face.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.cv.CV_HAAR_SCALE_IMAGE )
		
		for (x, y, w, h) in faces:
			cv2.rectangle(cv_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
		
		
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
		
		
		
