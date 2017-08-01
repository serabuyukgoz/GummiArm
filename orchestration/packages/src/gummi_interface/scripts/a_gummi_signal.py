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

		self.gummi = Gummi()
		self.r = rospy.Rate(60)

		self.name = name
		self.index = 0
		self.lists = [0.0]
		self.flag = False
		self.jointName = self.initJointNames(name)

		rospy.Subscriber("/" + self.name + "_encoder_controller/state", JointState, self.Callback)

		self.motionArray = ([0.0]*10)
		self.motionIndex = 0
		self.hitFlag = False
		self.data = 0.0
		

	def initJointNames(self, name):
		return {
			'elbow' : self.gummi.elbow,
			'wrist' : self.gummi.wrist,
			'shoulder_yaw' : self.gummi.shoulderYaw,
			'shoulder_roll' : self.gummi.shoulderRoll,
			'shoulder_pitch' : self.gummi.shoulderPitch,
		}[name]


	def Callback(self, msg):
		self.filterData(msg.velocity)	

	def filterData(self, data):
		if abs(data) < 0.1:
			if self.index == 0:
				data = 0.0	

		#self.checkMotion(data)

		if self.flag == True:
			self.findWave(data)	

	def findWave(self, data):
		if data != 0.0:
			self.lists.append(data)
			self.index += 1
		else:
			if self.index > 0:
				old = self.checkOld()
				if old == True:
					self.lists.append(data)
					self.index += 1
				else:
					self.setFlag(False)
					self.findAmplitude(self.lists)
					

	def checkOld(self):		
		if self.index > 4:
			if self.lists[self.index-2] == 0.0:
				return False
			else:
				return True
		else:
			return True

	def findAmplitude(self, array):
		
		print("==================" + self.name)
		print(array)
		maxEle = max(array)
		minELe = min(array)

		amplitude = abs(maxEle - minELe) / 2

		magnitude = amplitude / 100

		#cheking first element for direcion
		if array[1] > 0.0:
			print("UP")
			direction = magnitude
		elif array[1] < 0.0:
			print("Down")
			direction = -1 * magnitude

		print("Amplitude:  " + str(amplitude))
		print(direction)
		print("================")
		
		self.move(direction)

	def move(self, mag):
		
		print("Reflex Activated")
		for i in range(0,100):
			self.jointName.moveWith(mag, 0.5)
    		self.r.sleep()

		print("End of Reflex Motion")
		self.clear()

		self.setFlag(True)
		print("move: " + str(self.flag))

	def setFlag(self, data):
		self.flag = data

	def clear(self):
		self.index = 0
		self.lists = [0.0]

def main(args):

    pi = 3.1416

    rospy.init_node('GummiArm', anonymous=True)
    r = rospy.Rate(60)  
    p = 0

    reflex = Reflex("shoulder_roll")

    #After first motiom

    print('WARNING: Moving joints sequentially to equilibrium positions.')
    reflex.gummi.doGradualStartup()

    
    print('WARNING: Moving to resting pose, hold arm!')
    rospy.sleep(1)

    reflex.gummi.goRestingPose(True)

    for i in range(0,400):
        reflex.gummi.goRestingPose(False)
        r.sleep()



    print("GummiArm is live!")

    for i in range(0,300):
    	reflex.gummi.shoulderRoll.servoTo(pi/3, 0.6)
    	reflex.gummi.elbow.servoTo(-pi/4, 0.6)
    	r.sleep()

    print("Gummi in Rest Position")

    rospy.sleep(1)
    
    reflex.setFlag(True)

    while not rospy.is_shutdown():

		# if reflex.gummi.teleop == 0:
		# 	reflex.gummi.doUpdate()

		reflex.gummi.publishJointState()
		r.sleep()
  
if __name__ == '__main__':
	main(sys.argv)
