#!/usr/bin/env python

import rospy, time
import numpy as np
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from math import pi

import glob
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32

import os

def safeTime():
	while True:
		time = rospy.Time.now()
		if time != rospy.Time(0):
			return time

class Localization:
	def __init__(self):
		self.twistPub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size = 1)
		self.rate = rospy.Rate(10)

		rospy.wait_for_service('global_localization')
		# From http://wiki.ros.org/amcl
		self.global_localization = rospy.ServiceProxy('gobal_localization', Empty)
		self.imageSub = rospy.Subscriber('/camera/rgb/raw', Image, self.imageCallback)

	def start(self):
		print "in start"
		self.global_localization()
		time.sleep(0.5)

		duration = 12
		speed = 0.8
		twist = Twist()
		timeLimit = safeTime() + rospy.Duration(duration)
		while getTimeState() < timeLimit:
			twist.angular.z = speed
			self.twistPub.publish(tw)

		# self.clear_costmaps()
		print "done"

	def imageCallback(self, msg):
		print "something"
		scan_data = msg.ranges
		self.central_range = scan_data[320]


class emblemDetection:
	def __init__(self):
		self.directory = os.path.dirname(os.path.abspath(__file__))
		self.targetImage = cv2.imread(self.directory + "/emblem.png",0)

	def draw(self,img, imgpts):
		corner = tuple(imgpts[3].ravel())
		cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5) # blue
		cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5) # green
		cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5) # red
		return img


if __name__ == "__main__":
	rospy.init_node('nav')
	localization = Localization().start()

rospy.spin()

