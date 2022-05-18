#!/usr/bin/env python
import rospy
from utilities import Util, GoToPose
import cv2
from smach import State, StateMachine


# from utilities import Util, GoToPose

from states import ApproachPosition, WallFollow, LookAround, CheckSign


# main
def main():
    rospy.init_node('state_machine')
    
    next_location = 'room2_entrance'
    rospy.set_param('/next_location', next_location)
    rospy.set_param('/flag_green', False)
    rospy.set_param('/flag_red', False)


    # Create a SMACH state machine
    sm = StateMachine(outcomes=['end', 'end'])
    # Open the container

    with sm:
        # Add states to the container

        StateMachine.add('approach_position', ApproachPosition(), transitions={'outcome1':'check_sign', 'outcome2': 'look_around'})
        StateMachine.add('check_sign', CheckSign(), transitions={'outcome1':'approach_position', 'outcome2': 'end'})
        #StateMachine.add('wall_follow', WallFollow(), transitions={'outcome1':'end', 'outcome2': 'end'})
        StateMachine.add('look_around', LookAround(), transitions={'outcome1':'end', 'outcome2': 'end'})

        sm.execute()
    


    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('State Machine terminated...')
