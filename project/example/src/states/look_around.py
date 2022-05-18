#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import math
from utilities import Identifier

from smach import State


class LookAround(State):

    def __init__(self):
        rospy.loginfo('LookAround state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])

        self.pub_ = None
        self.rate_ = None
        self.turn_360_flag = True
        self.counter_360 = 1
        self.dist = 1

        
    def go_forward(self, distance):
        print distance
        velocity = Twist()
        velocity.linear.x = 0.5
        loop = int((10/0.5)*distance)
        print(loop)
        print(distance)
        for i in range(loop):
            self.character = self.character_detector.get_character()
            if self.character is not 'None':
                print ("The character identified is " + self.character)
                return True
            else:
                print ("No character can be found in the frame")
            self.pub_.publish(velocity)
            self.rate_.sleep()
        velocity.linear.x = 0
        self.pub_.publish(velocity)
        self.rate_.sleep()
        return 

    def go_back(self, distance):
        velocity = Twist()
        velocity.linear.x = -0.5
        loop = int((10/0.5)*distance)
        print(loop)
        print(distance)
        for i in range(loop):
            self.pub_.publish(velocity)
            self.rate_.sleep()
        velocity.linear.x = 0
        self.pub_.publish(velocity)
        self.rate_.sleep()
        return 
        
    def look_around(self):
        velocity = Twist()
        velocity.angular.z = -2.0944/4
        for i in range(20): #2sec; total angle = -60*
            self.character = self.character_detector.get_character()
            if self.character is not 'None':
                print ("The character identified is " + self.character)
                return True
            else:
                print ("No character can be found in the frame")
            self.pub_.publish(velocity)
            self.rate_.sleep()
        velocity.angular.z = 0
        self.pub_.publish(velocity)
        velocity.angular.z = 2.0944/4
        for i in range(40): #4sec; total angle = 120*
            self.character = self.character_detector.get_character()
            if self.character is not 'None':
                print ("The character identified is " + self.character)
                return True
            else:
                print ("No character can be found in the frame")
            self.pub_.publish(velocity)
            self.rate_.sleep()
        velocity.angular.z = 0
        self.pub_.publish(velocity)
        velocity.angular.z = -2.0944/4
        for i in range(20): #2sec; total angle = -60*
            self.character = self.character_detector.get_character()
            if self.character is not 'None':
                print ("The character identified is " + self.character)
                return True
            else:
                print ("No character can be found in the frame")
            self.pub_.publish(velocity)
            self.rate_.sleep()
        return

    def turn_360(self):
        velocity = Twist()
        velocity.angular.z = 6.28319/10
        for i in range(100): #2sec; total angle = 360*
            self.character = self.character_detector.get_character()
            if self.character is not 'None':
                print ("The character identified is " + self.character)
                return True
            else:
                print ("No character can be found in the frame")
            self.pub_.publish(velocity)
            self.rate_.sleep()
        velocity.angular.z = 0
        self.pub_.publish(velocity)
        return    

    def turn_45(self):
        velocity = Twist()
        velocity.angular.z = 0.785398/2
        for i in range(20): #2sec; total angle = 45*
            self.pub_.publish(velocity)
            self.rate_.sleep()
        velocity.angular.z = 0
        self.counter_360 = 1 + self.counter_360
        print self.counter_360
        self.pub_.publish(velocity)

    def stop(self):
        velocity = Twist()
        velocity.linear.x = 0.5
        return

    def search(self):
        if(self.counter_360 % 8 == 0): # 8 * 45 = 360
            self.dist = 0.5 + self.dist # increase travelled distance
        if self.turn_360_flag:
            self.turn_360()
            self.turn_360_flag = False
        if self.go_forward(self.dist) == True:
            return
        if self.look_around() == True:
            return True
        self.go_back(self.dist)



    def publish(self):
        
        print("start")
        self.pub_
        self.pub_ = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.rate_ = rospy.Rate(10) #10hz
        velocity  = Twist()
        #self.turn_360()
        
        while not rospy.is_shutdown():

                if self.search() == True:
                    return True
                self.turn_45()

        
    def execute(self, userdata):
        rospy.loginfo('LookAround state executing')

        # Instantiating character identifier
        self.character_detector = Identifier()

        self.publish()

        
        return 'outcome1'
