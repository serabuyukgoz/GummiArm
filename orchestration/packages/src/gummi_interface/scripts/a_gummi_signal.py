#!/usr/bin/env python

import rospy
import sys
import random
import time
from math import pi

from gummi_interface.gummi import Gummi
from dynamixel_msgs.msg import JointState

s = 0
t = False

def Callback(msg):
    global t
    global s
    hit_signal = msg.load
        #print "Load: " + str(hit_signal)

    if t == True:
        if  hit_signal >= 0.5:
            print ("Hit from left <--")
            t = False
            s = 1

        if hit_signal <= -0.5:
            print ("Hit from right --> ")
            t = False
            s = 2
         

def main(args):
    global t
    global s

    pi = 3.1416

    rospy.init_node('GummiArm', anonymous=True)
    r = rospy.Rate(60)  

    rospy.Subscriber("/shoulder_pitch_encoder_controller/state", JointState, Callback)

    gummi = Gummi()

    print('WARNING: Moving joints sequentially to equilibrium positions.')
    gummi.doGradualStartup()
    
    print('WARNING: Moving to resting pose, hold arm!')
    rospy.sleep(1)

    gummi.goRestingPose(True)
    for i in range(0,400):
        gummi.goRestingPose(False)
        r.sleep()

    gummi.setCollisionResponses(False, False, False, False, False)
    print("GummiArm is live!")

    for i in range(0,100):
        gummi.shoulderRoll.servoTo(pi/3, 0.6)
        r.sleep()

 	
    print("Try servoTo for shoulderpithch")

    t = True

    for i in range(0,100):
        gummi.elbow.servoTo(-pi/2, 0.6)
        r.sleep()

    while not rospy.is_shutdown():

       # if gummi.teleop == 0:
       #     gummi.doUpdate()

        if s == 1:
            for i in range(0,100):
                gummi.shoulderYaw.servoTo(-pi/3, 0.6)
                r.sleep()

            rospy.sleep(1)
            t = True

        if s == 2:
            for i in range(0,100):
                gummi.shoulderYaw.servoTo(pi/3, 0.6)
                r.sleep()
            rospy.sleep(1)
            t = True

        gummi.publishJointState()
        r.sleep()
  
if __name__ == '__main__':
	main(sys.argv)
