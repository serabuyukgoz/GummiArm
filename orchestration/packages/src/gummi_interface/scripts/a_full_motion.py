#!/usr/bin/env python

import sys
import rospy
import random
import time
from math import pi
import numpy as np

from gummi_interface.gummi import Gummi
from gummi_interface.msg import HitDetectReflex

from dynamixel_msgs.msg import JointState
from std_msgs.msg import Int8

####for moveit
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class ReflexMotion:
    def __init__(self):

        self.is_running = False
        self.moving_flag = False

        self.hit_flag = False
        self.is_motion_cont = True
        self.run_time = rospy.get_rostime()

        rospy.Subscriber('hit_signal', HitDetectReflex, self.hitFlagCall)

        rospy.Subscriber("/shoulder_pitch_encoder_controller/state", JointState, self.Callback)
        rospy.Subscriber("/shoulder_roll_encoder_controller/state", JointState, self.Callback)
        rospy.Subscriber("/shoulder_yaw_encoder_controller/state", JointState, self.Callback)
        rospy.Subscriber("/elbow_encoder_controller/state", JointState, self.Callback)
        rospy.Subscriber("/wrist_encoder_controller/state", JointState, self.Callback)

        self.pubHit = rospy.Publisher('hit_signal', HitDetectReflex, queue_size=1, latch=True)

    def hitFlagCall(self, msg):
        if msg.hit == False:
            self.run_time = rospy.get_rostime()

    def Callback(self, msg):

        if self.is_motion_cont == True:
            return

        if self.is_running == True:
            current = rospy.get_rostime()
            delay = current - self.run_time
            if delay.to_sec() > 1 and delay.to_sec() < 1.3 :
                self.hit_flag = False

        if abs(msg.velocity) > 0.15 and self.hit_flag == False:
            self.is_running = True
            self.run_time = rospy.get_rostime()
            self.hit_flag = True
            rospy.loginfo(msg.velocity)
            self.findAmplitude(msg.velocity, msg.name)

    def findAmplitude(self, vel, name):
        amplitude = abs(vel)
        direction = np.sign(vel)
        magnitude = amplitude*direction
        rospy.logwarn(str(magnitude) + str(name))
        self.hitPublish(name, amplitude, magnitude, direction)

    def hitPublish(self, name, amp, mag, direction):
        hitmsg = HitDetectReflex()
        hitmsg.name = name
        hitmsg.hit = True
        hitmsg.amplitude = amp
        hitmsg.direction = direction
        hitmsg.magnitude = mag
        hitmsg.time = rospy.get_rostime()
        self.pubHit.publish(hitmsg)

def main(args):

    rospy.init_node('GummiReflex', anonymous=False)
    r = rospy.Rate(60)

    gummi = Gummi()
    reflex = ReflexMotion()

    rospy.logwarn('Moving joints sequentially to equilibrium positions.')
    gummi.doGradualStartup()

    rospy.logwarn('Moving to resting pose, hold arm!')
    rospy.sleep(1)

    gummi.goRestingPose(True)
    for i in range(0,400):
        gummi.goRestingPose(False)
        r.sleep()

    gummi.setCollisionResponses(False, False, False, False, False)
    rospy.loginfo("GummiArm is live!")
    print("A Full body motion hit detection")

    reflex.is_motion_cont = False

    while not rospy.is_shutdown():
        if gummi.teleop == 0 and gummi.velocity_control == 0:
            gummi.doUpdate()

        gummi.publishJointState()
        r.sleep()

if __name__ == '__main__':
    main(sys.argv)
