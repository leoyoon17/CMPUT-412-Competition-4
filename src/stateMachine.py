#!/usr/bin/env python
import rospy
import time
from smach import State, StateMachine
from Localization import Localization
from Exploration import Exploration

if __name__ == "__main__":
	rospy.init_node('state_machine_controller')
	sm = StateMachine(outcomes=['success', 'markerFound','done'])
	with sm:
		StateMachine.add('Localization', Localization(),
						transitions={'success': 'Exploration'})
		StateMachine.add('Exploration', Exploration(),
						transitions={'markerFound': None, 'done': None})
	time.sleep(1)		
	sm.execute()