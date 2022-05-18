#!/usr/bin/python
import rospy
import yaml
import os

class Util:
    def __init__(self):

        self.load_input_points()
    
    
    def load_input_points(self):
        #path = file="~/catkin_ws/src/group_project/project/example/input_points_small_test_2.yaml"
        #path=os.getcwd()+file
        with open("input_points.yaml", 'r') as stream:
            points = yaml.safe_load(stream)
            self.room1_entrance_xy = points['room1_entrance_xy']
            self.room2_entrance_xy = points['room2_entrance_xy']
            self.room1_centre_xy = points['room1_centre_xy']
            self.room2_centre_xy = points['room2_centre_xy']



