#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

pub_ = None
rate_ = None


def go_forward(distance):
    global pub_, rate_
    velocity = Twist()
    velocity.linear.x = 0.5
    loop = (10/0.5)*distance
    print(int(loop))
    print(distance)
    for i in range(int(loop)):
        pub_.publish(velocity)
        rate_.sleep()
    velocity.linear.x = 0
    pub_.publish(velocity)
    rate_.sleep()
    return 

def go_back(distance):
    global pub_, rate_
    velocity = Twist()
    velocity.linear.x = -0.5
    loop = (10/0.5)*distance
    print(int(loop))
    print(distance)
    for i in range(int(loop)):
        pub_.publish(velocity)
        rate_.sleep()
    velocity.linear.x = 0
    pub_.publish(velocity)
    rate_.sleep()
    return 
    
def look_around():
    global pub_, rate_
    velocity = Twist()
    velocity.angular.z = -2.0944/4
    for i in range(20): #2sec; total angle = -60*
        pub_.publish(velocity)
        rate_.sleep()
    velocity.angular.z = 0
    pub_.publish(velocity)
    velocity.angular.z = 2.0944/4
    for i in range(40): #4sec; total angle = 120*
        pub_.publish(velocity)
        rate_.sleep()
    velocity.angular.z = 0
    pub_.publish(velocity)
    velocity.angular.z = -2.0944/4
    for i in range(20): #2sec; total angle = -60*
        pub_.publish(velocity)
        rate_.sleep()
    return

def turn_360():
    global pub_, rate_
    velocity = Twist()
    velocity.angular.z = 6.28319/10
    for i in range(100): #2sec; total angle = 360*
        pub_.publish(velocity)
        rate_.sleep()
    velocity.angular.z = 0
    pub_.publish(velocity)
    return    

def turn_45():
    global pub_, rate_
    velocity = Twist()
    velocity.angular.z = 0.785398/2
    for i in range(20): #2sec; total angle = 45*
        pub_.publish(velocity)
        rate_.sleep()
    velocity.angular.z = 0
    pub_.publish(velocity)

def stop():
    global pub_, rate_
    velocity = Twist()
    velocity.linear.x = 0.5
    return

def search():
    dist = 1
    go_forward(dist)
    look_around()
    go_back(dist)
    
def publisher():
    global pub_, rate_
    pub_ = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.init_node('wall_follow', anonymous=True)
    rate_ = rospy.Rate(10) #10hz
    velocity  = Twist()
    turn_360()
    while not rospy.is_shutdown():
        search()
        turn_45()
    
            
if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
