#!/usr/bin/env python

import rospy
import sys
import random
import time
from math import pi

from gummi_interface.gummi import Gummi

def main(args):

    rospy.init_node('GummiArm', anonymous=True)
    r = rospy.Rate(60)

    gummi = Gummi()

    rospy.logwarn('Moving joints sequentially to equilibrium positions.')
    gummi.doGradualStartup()

    rospy.logwarn('Moving to resting pose, hold arm!')
    rospy.sleep(1)

    gummi.goRestingPose(True)
    for i in range(0,400):
        gummi.goRestingPose(False)
        r.sleep()

    for i in range(0,100):
        gummi.shoulderRoll.moveWith(0.002, 0.05)
        r.sleep()

    gummi.setCollisionResponses(False, False, False, False, False)
    rospy.loginfo("GummiArm is live!")

    while not rospy.is_shutdown():
#        if gummi.teleop == 0 and gummi.velocity_control == 0:
#            gummi.doUpdate()

        gummi.setReflexFlag()

        gummi.publishJointState()
        r.sleep()

if __name__ == '__main__':
    main(sys.argv)
