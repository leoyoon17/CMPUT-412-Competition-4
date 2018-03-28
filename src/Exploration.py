#!/usr/bin/env python
import rospy
import os
import time
import math
import actionlib
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# Orientation Variables
cone1 = (0.0, 0.0, -0.908091001459, 0.418772889606)
cone2 = (0.0, 0.0, -0.28695816815,  0.957943114038)
cone3 = (0.0, 0.0, 0.492298195017,  0.870426612175)
cone4 = (0.0, 0.0, 0.970629644687, 0.240578662509)
finish_orientation = (0.0, 0.0,-0.704462374067,0.709741335646)

# Expected
# A list of waypoints for the robot to navigate around
waypoints = [[(0.135687176908, 2.87663573268, 0.0 ) , cone1],
		[(0.180769769101, 0.444637976371, 0.0), cone2],
		[(6.04934212359, 1.27606519063, 0.0), cone3],
		[( 5.29041238011, 3.86765393351, 0.0), cone4]]

finish = [[(-0.239126570389, 1.84109961989, 0.0) , finish_orientation]]

def goal_pose(pose):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = 'map'
	goal_pose.target_pose.pose.position.x = pose[0][0]
	goal_pose.target_pose.pose.position.y = pose[0][1]
	goal_pose.target_pose.pose.position.z = pose[0][2]
	goal_pose.target_pose.pose.orientation.x = pose[1][0]
	goal_pose.target_pose.pose.orientation.y = pose[1][1]
	goal_pose.target_pose.pose.orientation.z = pose[1][2]
	goal_pose.target_pose.pose.orientation.w = pose[1][3]
	return goal_pose

def getTimeSafe():
	while True:
		# rospy may returns zero, so we loop until get a non-zero value.
		time = rospy.Time.now()
		if time != rospy.Time(0):
			return time

class Exploration(State):
	def __init__(self):
		State.__init__(self, outcomes=['markerFound', 'done'])
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client.wait_for_server()

		self.markerFonud = False

		self.directory = os.path.dirname(os.path.abspath(__file__))
		# self.soundSource = self.directory + '/wilhelm.wav'
		self.soundSource = self.directory + '/yay.wav'
		print self.soundSource
		self.soundHandle = SoundClient()
		self.currentPosition = None

	def execute(self, userdata):
		print('moving to first point')
	
		# Start by going to the first cone/waypoint
		self.client.send_goal(goal_pose(waypoints[0]))
		self.client.wait_for_result()

		print('at first point')
		self.soundHandle.playWave(self.soundSource)


		# for pose in waypoints:
		# 	goal = goal_pose(pose)
		# 	self.client.send_goal(goal)
		# 	self.client.wait_for_results()
		# print "done one lap."
		return 'done'