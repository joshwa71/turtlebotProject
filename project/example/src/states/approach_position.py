#!/usr/bin/env python
import rospy
import actionlib
import numpy as np


from utilities import Util, GoToPose
from smach import State
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped


class ApproachPosition(State):
    def __init__(self):
        rospy.loginfo('ApproachPosition state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
        self.util = Util()
        self.go_to_pose = GoToPose()

    def execute(self, userdata, wait=True):
        rospy.loginfo('ApproachPosition state executing')

        # self.util.room1_entrance_xy
        # self.util.room2_entrance_xy
        # self.util.room1_centre_xy
        # self.util.room2_centre_xy

        next_location = rospy.get_param("/next_location")
        self.location = next_location

        # Saving the parameter for current approach position.
        if self.location == 'room1_entrance':
            current_approach_position = self.util.room1_entrance_xy
        elif self.location == 'room2_entrance':
            current_approach_position = self.util.room2_entrance_xy
        elif self.location == 'room1_centre':
            current_approach_position = self.util.room1_centre_xy
        elif self.location == 'room2_centre':
            current_approach_position = self.util.room2_centre_xy
        print (self.location)
        theta = 0
        current_approach_position.append(theta)
        rospy.set_param('/location', self.location)
        rospy.set_param('/current_approach_position', [current_approach_position[0], current_approach_position[1], current_approach_position[2]])
        
        # Getting the parameter for current approach position
        current_approach_position = rospy.get_param("/current_approach_position")

        # saving pos and orientation data to pass to go_to_pose class
        x = current_approach_position[0]
        y = current_approach_position[1]
        theta = current_approach_position[2]
        position = {'x': x, 'y' : y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
        
        # Sending command to turtlebot to go to requested position
        self.go_to_pose.goto(position, quaternion)

        if (self.location == 'room1_centre') or (self.location == 'room2_centre'):
            return 'outcome2'
        else:
            return 'outcome1'
