#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math

state_ = 0
pub_ = None

regions_ ={
    'front_right': 0,
    'center': 0,
    'front_left': 0,
}

state_dict_ = {
    0:'find the wall',
    1:'turn left',
    2:'follow the wall',
    3:'go back'
}

def callback(msg):
    #print("sub call back")
    msg_len = len(msg.ranges)/5
    global regions_
    regions_ = {
        'front_right' : min(msg.ranges[0:msg_len/2]),
        'center' : min(msg.ranges[msg_len/2+1:5*msg_len-msg_len/2 -1]),
        'front_left' : min(msg.ranges[5*msg_len-msg_len/2:5*msg_len-1]),
    }
    if(math.isnan(regions_['center'])):
        regions_['center'] = 10
    if(math.isnan(regions_['front_right'])):
        regions_['front_right'] = 10
    if(math.isnan(regions_['front_left'])):
        regions_['front_left'] = 10
    follower()
    #range_center = msg.ranges[len(msg.ranges)/2]
    #range_left = msg.ranges[len(msg.ranges)-1]
    #range_right = msg.ranges[0]
    #print("range ahead: left - %0.1f" %regions_['front_left'], "center - %0.1f" %regions_['center'], "right - %0.1f" %regions_['front_right'])
    
def change_state(state):
    #print('state change')
    global state_, state_dict_
    if state is not state_:
        print ('wall follower: [%s] - %s' %(state, state_dict_[state]))
        state_ = state

def follower():
    global regions_
    regions = regions_
    #velocity = Twist()

    state_disc = ''
    
    distance = 1.5
    
    if(regions['center'] > distance and regions['front_right'] > distance and regions['front_left'] > distance):
        state_disc = 'case 0 - all clear'
        change_state(0)
    elif(regions['center'] < distance and regions['front_right'] > distance and regions['front_left'] > distance):
        state_disc = 'case 1 - something detected in front'
        change_state(1)
    elif(regions['center'] > distance and regions['front_right'] < distance and regions['front_left'] > distance):
        state_disc = 'case 2 - something detected in front_right'
        change_state(2)
    elif(regions['center'] > distance and regions['front_right'] > distance and regions['front_left'] < distance):
        state_disc = 'case 3 - something detected in front_left'
        change_state(0)
    elif(regions['center'] < distance and regions['front_right'] < distance and regions['front_left'] > distance):
        state_disc = 'case 4 - something detected in front and front right'
        change_state(1)
    elif(regions['center'] < distance and regions['front_right'] > distance and regions['front_left'] < distance):
        state_disc = 'case 5 - something detected in front and front left'
        change_state(1)
    elif(regions['center'] > distance and regions['front_right'] < distance and regions['front_left'] < distance):
        state_disc = 'case 6 - something detected in front right and front left'
        change_state(3)
    elif(regions['center'] < distance and regions['front_right'] < distance and regions['front_left'] < distance):
        state_disc = 'case 7 - front blocked'
        change_state(1)
    else:
        state_disc = 'unknown'
    
    print (state_disc)
        
def find_wall():
    velocity = Twist()
    velocity.linear.x = 0.2
    velocity.angular.z = -0.15
    return velocity

def turn_left():
    velocity = Twist()
    velocity.linear.x = 0
    velocity.angular.z = 1.0
    return velocity
    
def follow_the_wall():
    global regions_
    velocity = Twist()
    velocity.angular.z = 0
    velocity.linear.x = 0.6
    return velocity

def go_back():
    global regions_
    velocity = Twist()
    velocity.angular.z = 2
    velocity.linear.x = -1
    return velocity
    
def publish():
    print("start")
    global pub_
    rospy.init_node('scan_values', anonymous=True)
    pub_ = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    sub = rospy.Subscriber('kobuki/laser/scan', LaserScan, callback)
    rate = rospy.Rate(10) #10hz
    velocity = Twist()
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    while not rospy.is_shutdown():
        if state_ == 0:
            velocity = find_wall()
        elif state_ == 1:
            velocity = turn_left()
        elif state_ == 2:
            velocity = follow_the_wall()
        elif state_ == 3:
            velocity = go_back()
            pass
        else:
            print('fail')

        pub_.publish(velocity)
        rate.sleep()
    
if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass

