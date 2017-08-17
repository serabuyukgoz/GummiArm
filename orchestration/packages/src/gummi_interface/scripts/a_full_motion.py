#!/usr/bin/env python

import sys
import rospy
import random
import time
from math import pi

from gummi_interface.gummi import Gummi
from gummi_interface.msg import HitDetectReflex

####for moveit
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class ReflexMotion:
    def __init__(self):

        rospy.init_node('GummiArm', anonymous=True)
        self.r = rospy.Rate(60)
        self.gummi = Gummi()
        
        rospy.Subscriber('hit_signal', HitDetectReflex, self.Callback)

        self.is_running = False

    def Callback(self, data):
        if self.is_running:
            return
        self.is_running = True

        name = data.name
        hit = data.direction
        amplitude = data.amplitude
        print(name + " Data " + str(hit))
        self.path(hit, name, amplitude)
        
    def path(self, hit, name, amp):
         
        if hit < 0:
            print("UP")
            if name == 'elbow' or name == 'shoulder_roll':
                if amp > 0.15:
                    print("left top")
                    angles = [  0.5062136600022616, 
                                0.9612946270750019, 
                                -0.08692557798018634, 
                                -0.7286408742456796, 
                                0.0,
                                -0.010226538585904275, 
                                0.0, 
                                -0.16873788666742054]
                    self.move(angles)
                else:
                    print("left mid")
                    angles = [0.5931392379824479, 0.0, 0.0, -0.8513593372765309, 0.0, -0.010226538585904275, 0.0, -0.16873788666742054]
                    self.move(angles)
            else:
                print("left bottom")
                angles = [0.29145634969827183, -0.07158577010132992, 0.0, -0.4724660826687775, -0.8385761640441506, -0.010226538585904275, 0.10737865515199489, -0.16873788666742054]
                self.move(angles)
        elif hit > 0:
            if name == 'elbow' or name == 'shoulder_roll':
                print("right back")
                angles = [-0.41417481272912315, -0.4908738521234052, 0.4192880820220753, 1.2701360923693108, -0.8232363561652941, -0.010226538585904275, 0.3119094268700804, -0.16873788666742054] 
                self.move(angles)
            else:
                if amp > 0.15:
                    print("right bottom")
                    angles = [-0.260776733940559, -0.19430423313218123, 0.8385761640441506, -0.20708740636456155, -0.7311975088921556, -0.010226538585904275, 0.24032365676875048, -0.16873788666742054]
                    self.move(angles)
                else:
                    print("mid bottom")
                    angles = [-0.1227184630308513, -0.07158577010132992, 0.0, 0.15800002115222103, -0.782330201821677, -0.010226538585904275, 0.06647250080837779, -0.16873788666742054] 
                    self.move(angles)

    def move(self, msg):
        for i in range (0,100):
           # gummi.setCocontraction(0.6, 0.6, 0.6, 0.6, 0.6)
            self.gummi.goTo(msg, False)
            self.r.sleep()

        self.is_running = False

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
    print("A Full body motion with GoTo")

    while not rospy.is_shutdown():
      #  if gummi.teleop == 0 and gummi.velocity_control == 0:
      #      gummi.doUpdate()

        reflex.gummi.setReflexFlag()

        reflex.gummi.publishJointState()
        reflex.r.sleep()

if __name__ == '__main__':
    main(sys.argv)
