#!/usr/bin/env python
import rospy
import time
from smach import State, StateMachine

if __name == "__main__":
	rospy.init_node('state_machine_controller')
	sm = StateMachine(outcomes=['success'])
	with sm:
		StateMachine.add('Localization', Localization(),
						transitions={'success': 'Explore'})
		