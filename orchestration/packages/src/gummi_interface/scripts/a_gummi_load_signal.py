#!/usr/bin/env python

import rospy
import sys
import random
import time
from math import pi

from gummi_interface.gummi import Gummi

rest = (0.5,0.6632251157578453,0.5410520681182421,-0.12217304763960307,0.5585053606381855,-2.4085543677521746,-0.7330382858376184)
mid = (0.05,1.8,0.2792526803190927,-0.17453292519943295,0.4014257279586958,-2.3736477827122884,-0.7330382858376184)
desired = (0.1,1.8500490071139892,0.20943951023931956,-0.24434609527920614,-0.45,-2.7,-0.2617993877991494)

def main(args):

    rospy.init_node('GummiArm', anonymous=True)
    r = rospy.Rate(60)  

    gummi = Gummi()
	#contraction can be set 
	#gummi.setCocontraction(0.6, 0.6, 0.6, 0.6, 0.6)

    print('WARNING: Moving joints sequentially to equilibrium positions.')
    gummi.doGradualStartup()
    
    print('WARNING: Moving to resting pose, hold arm!')
    rospy.sleep(1)

    gummi.goRestingPose(True)
    for i in range(0,400):
        gummi.goRestingPose(False)
        r.sleep()

#No meaning for them
    #for i in range(0,100):
    #    gummi.forearmRoll.servoTo(pi/2)
    #    r.sleep()

    #gummi.setCollisionResponses(False, False, False, False, False)
    print("GummiArm is live!")
#Collision detection do nothing


#Main movement inside while loop
    while not rospy.is_shutdown():
    	for i in range (0,500):
			gummi.setCocontraction(0.6, 0.6, 0.6, 0.6, 0.6)
            gummi.goTo(rest, False)
            r.sleep()

        for i in range (0,60):
            gummi.setCocontraction(0.6, 0.6, 0.85, 0.6, 0.2)
            gummi.goTo(mid, False)
            r.sleep()
        
        for i in range (0,500):
            gummi.setCocontraction(0.8, 0.5, 1.0, 0.3, 0.2)
            gummi.goTo(desired, True)
            r.sleep()
       # if gummi.teleop == 0:
       #     gummi.doUpdate()

        #gummi.publishJointState()
        #r.sleep()
  
if __name__ == '__main__':
    main(sys.argv)
