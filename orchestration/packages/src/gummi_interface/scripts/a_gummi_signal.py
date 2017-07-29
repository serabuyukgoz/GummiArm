#!/usr/bin/env python

import rospy
import sys
import random
import time

import numpy as np 

from math import pi

from gummi_interface.gummi import Gummi
from dynamixel_msgs.msg import JointState

class Reflex:
	def __init__(self, name):
		self.name = name
		self.data = 0.0
		self.oldData = 0.0
		self.index = 0
		self.lists = [0.0]
		self.flag = False

		self.amplitude = 0.0
		self.hitFlag = False

		self.gummi = Gummi()
		self.r = rospy.Rate(60)

		rospy.Subscriber(self.name + "_controller/state", JointState, self.Callback)

	def Callback(self, msg):

		if self.flag == True:
			signal = msg.velocity
			print(signal)
			self.filterData(signal)		


	def filterData(self, data):

		if data > 0.1:
			data = data
		else:
			if self.index > 0:
				data = data
			else:
				data = 0.0

		self.findWave(data)
	

	def findWave(self, data):

		self.data = data

		if self.data > 0.0:
			self.lists.append(self.data)
			self.index += 1
			if self.index > 3:
				self.oldData = self.lists[self.index-2]
			else:
				self.oldData = self.data

		elif self.data == 0.0:
			if self.oldData == 0.0:
				if len(self.lists) > 1:
					self.findAmplitude(self.lists)
					self.index = 0
					self.lists = [0.0]
			else:
				self.lists.append(self.data)
				self.index += 1
				if self.index > 3:
					self.oldData = self.lists[self.index-2]
				else:
					self.oldData = self.data

	def findAmplitude(self, array):
		print(array)
		maxEle = max(array)
		minELe = min(array)

		self.amplitude = abs(maxEle - minELe) / 2
		#self.hitFlag = True

		print("Amplitude:  " + str(self.amplitude))
		#self.move()

	def move(self):
		self.flag = False
		print("Reflex Activated")
		for i in range(0,100):
			self.gummi.elbow.moveWith(0.002, 0.5)
			self.r.sleep()	
		print("End of Reflex Motion")
		self.flag = True	

	def setFlag(self, data):
		self.flag = data

	def getFlag(self):
		return self.hitFlag

	# def setJoint(self):

	# 	return jointName

def main(args):

    pi = 3.1416

    rospy.init_node('GummiArm', anonymous=True)
    r = rospy.Rate(60)  

    gummi = Gummi()
    reflex = Reflex("/elbow_encoder")

    #After first motiom

    print('WARNING: Moving joints sequentially to equilibrium positions.')
    #gummi.doGradualStartup()
    
    print('WARNING: Moving to resting pose, hold arm!')
    rospy.sleep(1)

    gummi.goRestingPose(True)

    for i in range(0,400):
        gummi.goRestingPose(False)
        r.sleep()


    print("GummiArm is live!")

    for i in range(0,100):
        gummi.shoulderRoll.servoTo(pi/3, 0.6)
        gummi.elbow.servoTo(-pi/4, 0.6)
        r.sleep()        

	gummi.setCollisionResponses(False, False, False, False, False)
    print("GummiArm is live!")
    rospy.sleep(1)

    reflex.setFlag(True)

    while not rospy.is_shutdown():


		if gummi.teleop == 0:
			gummi.doUpdate()

		# flag = reflex.getFlag()

		# if flag == True:
		# 	reflex.setFlag(False)
		#  	print("Start")

		#  	for i in range(0,100):
		# 		#gummi.shoulderRoll.servoTo(pi/3, 0.6)
		# 		gummi.elbow.moveWith(0.002, 0.5)
		# 		r.sleep()

		# 	reflex.setFlag(True) 
		# 	print("End of Reflex") 



		gummi.publishJointState()
		r.sleep()
  
if __name__ == '__main__':
	main(sys.argv)
