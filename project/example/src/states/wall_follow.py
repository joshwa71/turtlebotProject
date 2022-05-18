#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math
from utilities import Identifier

from smach import State


class WallFollow(State):

    def __init__(self):
        rospy.loginfo('WallFollow state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])


        self.state_ = 0
        self.pub_ = None

        self.regions_ ={
            'front_right': 0,
            'center': 0,
            'front_left': 0,
        }

        self.state_dict_ = {
            0:'find the wall',
            1:'turn left',
            2:'follow the wall',
            3:'go back',
            4:'look_right'
        }
        


    def callback(self, msg):
        
        #print("sub call back")
        msg_len = len(msg.ranges)/5
        self.regions_
        self.regions_ = {
            'front_right' : min(msg.ranges[0:msg_len/2]),
            'center' : min(msg.ranges[msg_len/2+1:5*msg_len-msg_len/2 -1]),
            'front_left' : min(msg.ranges[5*msg_len-msg_len/2:5*msg_len-1]),
        }
        if(math.isnan(self.regions_['center'])):
            self.regions_['center'] = 10
        if(math.isnan(self.regions_['front_right'])):
            self.regions_['front_right'] = 10
        if(math.isnan(self.regions_['front_left'])):
            self.regions_['front_left'] = 10
        self.follower()
        #range_center = msg.ranges[len(msg.ranges)/2]
        #range_left = msg.ranges[len(msg.ranges)-1]
        #range_right = msg.ranges[0]
        #print("range ahead: left - %0.1f" %self.regions_['front_left'], "center - %0.1f" %self.regions_['center'], "right - %0.1f" %self.regions_['front_right'])
        
    def change_state(self, state):
        #print('state change')
        self.state_, self.state_dict_
        if state is not self.state_:
            print ('wall follower: [%s] - %s' %(state, self.state_dict_[state]))
            self.state_ = state

    def follower(self):
        self.regions_
        regions = self.regions_
        velocity = Twist()

        self.state_disc = ''
        
        distance = 1.5
        
        if(regions['center'] > distance and regions['front_right'] > distance and regions['front_left'] > distance):
            self.state_disc = 'case 0 - all clear'
            self.change_state(0)
            #self.change_state(4)
        elif(regions['center'] < distance and regions['front_right'] > distance and regions['front_left'] > distance):
            self.state_disc = 'case 1 - something detected in front'
            self.change_state(1)
            #self.change_state(4)
        elif(regions['center'] > distance and regions['front_right'] < distance and regions['front_left'] > distance):
            self.state_disc = 'case 2 - something detected in front_right'
            self.change_state(2)
            #self.change_state(4)
        elif(regions['center'] > distance and regions['front_right'] > distance and regions['front_left'] < distance):
            self.state_disc = 'case 3 - something detected in front_left'
            self.change_state(0)
            #self.change_state(4)
        elif(regions['center'] < distance and regions['front_right'] < distance and regions['front_left'] > distance):
            self.state_disc = 'case 4 - something detected in front and front right'
            self.change_state(1)
            #self.change_state(4)
        elif(regions['center'] < distance and regions['front_right'] > distance and regions['front_left'] < distance):
            self.state_disc = 'case 5 - something detected in front and front left'
            self.change_state(1)
            #self.change_state(4)
        elif(regions['center'] > distance and regions['front_right'] < distance and regions['front_left'] < distance):
            self.state_disc = 'case 6 - something detected in front right and front left'
            self.change_state(3)
            #self.change_state(4)
        elif(regions['center'] < distance and regions['front_right'] < distance and regions['front_left'] < distance):
            self.state_disc = 'case 7 - front blocked'
            self.change_state(1)
            #self.change_state(4)
        else:
            self.state_disc = 'unknown'
        
        #print (self.state_disc)
            
    def find_wall(self):
        velocity = Twist()
        velocity.linear.x = 0.2
        velocity.angular.z = -0.2
        return velocity

    def turn_left(self):
        velocity = Twist()
        velocity.linear.x = 0
        velocity.angular.z = 1.0
        return velocity
        
    def follow_the_wall(self):
        self.regions_
        velocity = Twist()
        velocity.angular.z = 0
        velocity.linear.x = 0.6
        return velocity

    def go_back(self):
        self.regions_
        velocity = Twist()
        velocity.angular.z = 2
        velocity.linear.x = -1
        return velocity

    def look_right(self):
        self.regions_
        velocity = Twist()
        velocity.angular.z = -0.1
        velocity.linear.x = 0
        return velocity
        
    def publish(self):
        print("start")
        self.pub_
        #rospy.init_node('scan_values', anonymous=True)
        self.pub_ = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        sub = rospy.Subscriber('kobuki/laser/scan', LaserScan, self.callback)
        rate = rospy.Rate(5) #10hz
        velocity = Twist()
        sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        while not rospy.is_shutdown():
            self.character = self.character_detector.get_character()
            if self.character is not 'None':
                print ("The character identified is " + self.character)
            else:
                print ("No character can be found in the frame")

            if self.state_ == 0:
                velocity = self.find_wall()
            elif self.state_ == 1:
                velocity = self.turn_left()
            elif self.state_ == 2:
                velocity = self.follow_the_wall()
            elif self.state_ == 3:
                velocity = self.go_back()
            elif self.state_ == 4:
                velocity = self.look_right()
                pass
            else:
                print('fail')

            self.pub_.publish(velocity)
            rate.sleep()
        
    def execute(self, userdata, wait=True):
        rospy.loginfo('WallFollow state executing')

        # Instantiating character identifier
        self.character_detector = Identifier()

        self.publish()

        
        return 'outcome1'
