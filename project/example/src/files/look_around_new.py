#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def go_forward(distance):
    velocity = Twist()
    velocity.linear.x = 0.5
    loop = (0.5/10)*distance
    for i in range(int(loop)):
        return velocity
    velocity.linear.x = 0
    return velocity

def stop():
    velocity = Twist()
    velocity.linear.x = 0.5
    return velocity

def publisher():
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.init_node('go_around', anonymous=True)
    rate = rospy.Rate(10) #10hz
    velocity = Twist()
    velocity = go_forward(2)
    pub.publish(velocity)
    rate.sleep()
            
if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
