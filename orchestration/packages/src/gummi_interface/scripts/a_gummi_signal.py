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
		self.data = 0.0
		self.oldData = 0.0
		self.index = 0
		self.lists = [0.0]
		self.flag = False
		self.amplitude = 0.0
		self.jointName = self.initJointNames(name)

		rospy.Subscriber("/" + self.name + "_encoder_controller/state", JointState, self.Callback)

		self.motionArray = ([0.0]*10)
		self.motionIndex = 0
		self.hitFlag = False
		

	def initJointNames(self, name):
		return {
			'elbow_encoder' : self.gummi.elbow,
			'wrist_encoder' : self.gummi.wrist,
			'shoulder_yaw_encoder' : self.gummi.shoulderYaw,
			'shoulder_roll_encoder' : self.gummi.shoulderRoll,
			'shoulder_pitch_encoder' : self.gummi.shoulderPitch,
		}[name]


	def Callback(self, msg):

		signal = msg.velocity
		self.filterData(signal)		


	def filterData(self, data):

		if abs(data) > 0.1:
			data = data
		else:
			if self.index > 0:
				data = data
			else:
				data = 0.0

		
		
		#self.data = data

		#self.checkMotion(data)

		if self.flag == True:
			self.findWave(data)
	

	def findWave(self, data):		

		if data != 0.0:
			self.lists.append(data)
			self.index += 1
			if self.index > 3:
				self.oldData = self.lists[self.index-2]
			else:
				self.oldData = data

		elif data == 0.0:
			if self.oldData == 0.0:
				if len(self.lists) > 1:
					self.findAmplitude(self.lists)
					self.setFlag(False)
					self.clear()
			else:
				self.lists.append(data)
				self.index += 1
				if self.index > 3:
					self.oldData = self.lists[self.index-2]
				else:
					self.oldData = data

	def findAmplitude(self, array):
		
		print("==================" + self.name)
		#print(array)
		maxEle = max(array)
		minELe = min(array)

		amplitude = abs(maxEle - minELe) / 2

		magnitude = amplitude / 100


		if array[1] > 0.0:
			print("UP")
			direction = magnitude
		elif array[1] < 0.0:
			print("Down")
			direction = -1 * magnitude

		print("-------" + str(array[:5]))
		print("Amplitude:  " + str(amplitude))
		print(direction)
		print("================")
		
		#self.move(direction)

	def move(self, mag):
		
		print("Reflex Activated")
		for i in range(0,100):
			self.jointName.moveWith(mag, 0.5)
    		self.r.sleep()
    		print(mag)

		print("End of Reflex Motion")

		# while hitFlag == False:
		# 	print("?????????")
		# 	hitFlag = self.check()

		self.setFlag(True)	

	def setFlag(self, data):
		if data == False:
			self.clear()
		self.flag = data

	def clear(self):
		self.index = 0
		self.lists = [0.0]
		self.oldData = 0.0

	# def checkMotion(self, data):

	# 	self.motionArray[self.motionIndex] = data


	# 	if self.motionIndex < 9:
	# 		self.motionIndex += 1
	# 	elif self.motionIndex == 9:
	# 		self.motionIndex = 0
	# 		ave = np.mean(self.motionArray)
	# 		#print(ave)
	# 		if ave == 0.0:
	# 			self.hitFlag = True
	# 		else:
	# 			self.hitFlag = False
	# 		#print(self.hitFlag)

	# def check(self):
	# 	return self.hitFlag


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

		if reflex.gummi.teleop == 0:
			reflex.gummi.doUpdate()

		reflex.gummi.publishJointState()
		r.sleep()
  
if __name__ == '__main__':
	main(sys.argv)
