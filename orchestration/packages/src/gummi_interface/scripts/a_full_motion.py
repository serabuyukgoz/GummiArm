#!/usr/bin/env python

import rospy
import sys
import random
import time
from math import pi

from gummi_interface.gummi import Gummi
from gummi_interface.msg import HitDetectReflex

#defined positions
#    lt = (0.7158577010132993, 1.5186409800067848, -0.12783173232380343, -0.7761942786701345, 0.0, -0.010226538585904275, -0.15851134808151626, -2.3674436826368397)
#    lm = (0.6647250080837779, 0.02045307717180855, -0.10226538585904275, -0.8022719520641903, 0.0, -0.010226538585904275, -0.18407769454627695, -2.3674436826368397)
#    lb = (0.4857605828304531, -0.1227184630308513, -0.05113269292952138, -0.4663301595172349, -0.5880259686894959, -0.010226538585904275, -0.06647250080837779, -2.3674436826368397)
#    mb = (-0.4192880820220753, -0.015339807878856412, 0.07158577010132992, -0.29299033048615747, -0.8385761640441506, -0.010226538585904275, 0.0, -2.3674436826368397)
#    rm = (-0.17896442525332482, -0.3119094268700804, 0.1227184630308513, 1.55545651891604, 0.0, -0.015339807878856412, 0.37326865838550605, -0.36304211979960177)
#    rt = (-0.5062136600022616, -0.07158577010132992, 1.0277671278833795, -1.3314953238847365, -0.0051132692929521375, -0.0051132692929521375, -0.09715211656609062, -0.3272492347489368)
#    bt = (0.010226538585904275, -1.3959225169759335, 0.0051132692929521375, -0.0046019423636569235, 0.02045307717180855, -0.010226538585904275, 0.015339807878856412, -0.06647250080837779)   


class ReflexMotion:
    def __init__(self):

        rospy.init_node('GummiArm', anonymous=True)
        self.r = rospy.Rate(60)
        self.gummi = Gummi()
        
        rospy.Subscriber('hit_signal', HitDetectReflex, self.Callback)
    
    def Callback(self, data):
        hit = data.direction
        self.decidePath(hit)
        
    def decidePath(self, hit):
        if hit > 0:
            msg = (-0.5062136600022616, -0.07158577010132992, 1.0277671278833795, -1.3314953238847365, -0.0051132692929521375, -0.0051132692929521375, -0.09715211656609062, -0.3272492347489368)
            self.move(msg)
        else:  
            msg = (0.010226538585904275, -1.3959225169759335, 0.0051132692929521375, -0.0046019423636569235, 0.02045307717180855, -0.010226538585904275, 0.015339807878856412, -0.06647250080837779)   
            self.move(msg)

    def move(self, msg):
        for i in range (0,500):
           # gummi.setCocontraction(0.6, 0.6, 0.6, 0.6, 0.6)
            self.gummi.goTo(msg, False)
            self.r.sleep()    

def main(args):

    reflex = ReflexMotion()

    rospy.logwarn('Moving joints sequentially to equilibrium positions.')
    reflex.gummi.doGradualStartup()

    rospy.logwarn('Moving to resting pose, hold arm!')
    rospy.sleep(1)

    reflex.gummi.goRestingPose(True)
    for i in range(0,400):
        reflex.gummi.goRestingPose(False)
        reflex.r.sleep()

    reflex.gummi.setCollisionResponses(False, False, False, False, False)
    rospy.loginfo("GummiArm is live!")
    

    while not rospy.is_shutdown():
      #  if gummi.teleop == 0 and gummi.velocity_control == 0:
      #      gummi.doUpdate()

        reflex.gummi.setReflexFlag()

        reflex.gummi.publishJointState()
        reflex.r.sleep()

if __name__ == '__main__':
    main(sys.argv)
