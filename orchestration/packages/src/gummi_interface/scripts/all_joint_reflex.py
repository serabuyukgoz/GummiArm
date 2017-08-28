#!/usr/bin/env python

import rospy
import sys
import random
import time
from math import pi

from gummi_interface.gummi import Gummi
from gummi_interface.msg import HitDetectReflex

from dynamixel_msgs.msg import JointState
from std_msgs.msg import Int8

class JointByJoint():
    def __init__(self):

        self.reflex_joint = {"shoulder_pitch_encoder": 0,
        "shoulder_roll_encoder": 0,
        "shoulder_yaw_encoder": 0,
        "elbow_encoder": 0,
        "wrist_encoder": 0 }

        rospy.Subscriber("/shoulder_pitch_encoder_controller/state", JointState, self.Callback)
        rospy.Subscriber("/shoulder_roll_encoder_controller/state", JointState, self.Callback)
        rospy.Subscriber("/shoulder_yaw_encoder_controller/state", JointState, self.Callback)
        rospy.Subscriber("/elbow_encoder_controller/state", JointState, self.Callback)
        rospy.Subscriber("/wrist_encoder_controller/state", JointState, self.Callback)

        pubHit = rospy.Publisher('hit_signal', HitDetectReflex, queue_size=1, latch=True)

        self.elbowdeg = 0.0
        self.wristdeg = 0.0
        self.rolldeg = 0.0
        self.pitchdeg = 0.0
        self.yawdeg = 0.0
        self.hit = False
        self.listenHit = False
        self.timeCheck = -1
        self.tt = rospy.get_rostime()

    def Callback(self, msg):

        current = rospy.get_rostime()

        if self.listenHit == False:
            return

        if abs(msg.velocity) > 0.15 and self.hit == False:
            if self.reflex_joint[msg.name] == 0:
                self.reflex_joint[msg.name] = msg.velocity
                self.timeCheck += 1
            #time giving errors
        if self.timeCheck == 0:
            self.tt = rospy.get_rostime()

        if self.timeCheck > 0:
            delay = current - self.tt

            if delay.to_sec() > 0.25:
                print("stop listen")
                self.listenHit = False
                self.findAmplitude()


    def findAmplitude(self):
        self.elbowdeg = self.reflex_joint["elbow_encoder"] / 100
        self.wristdeg = self.reflex_joint["wrist_encoder"] / 100
        self.rolldeg = self.reflex_joint["shoulder_roll_encoder"] / 100
        self.pitchdeg = self.reflex_joint["shoulder_pitch_encoder"] / 100
        self.yawdeg = self.reflex_joint["shoulder_yaw_encoder"] / 100
        self.hit = True
        self.reflex_joint["elbow_encoder"] = 0
        self.reflex_joint["wrist_encoder"] = 0
        self.reflex_joint["shoulder_roll_encoder"] = 0
        self.reflex_joint["shoulder_pitch_encoder"] = 0
        self.reflex_joint["shoulder_yaw_encoder"] = 0
        self.timeCheck = -1
        rospy.loginfo("Elbow : " + str(self.elbowdeg) +
                        "-- Wrist : " + str(self.wristdeg) +
                        "-- Roll: " + str(self.rolldeg) +
                        "-- Pitch : " + str(self.pitchdeg) +
                        "-- Yaw : " + str(self.yawdeg))

def main(args):

    rest = (0.26589000323351114, 0.3681553890925539, 0.20964404101103765, -0.6688156235181395, 0.1380582709097077, -0.0409061543436171, 0.22498384888989406, -0.2505501953546547)
    back = (0.0, -0.9919742428327146, 0.0, 0.0030679615757712823, 0.0, -0.010226538585904275, -0.08692557798018634, -0.15851134808151626)
    lie = (-0.0051132692929521375, 0.966407896367954, 0.0, -0.0046019423636569235, -0.6442719309119693, -0.04601942363656924, 0.3170226961630325, -0.19941750242513337)

    rospy.init_node('GummiArm', anonymous=True)
    r = rospy.Rate(60)
    gummi = Gummi()
    reflex = JointByJoint()

    rospy.logwarn('Moving joints sequentially to equilibrium positions.')
    gummi.doGradualStartup()

    rospy.logwarn('Moving to resting pose, hold arm!')
    rospy.sleep(1)

    gummi.goRestingPose(True)
    for i in range(0,400):
        gummi.goRestingPose(False)
        r.sleep()

    for i in range (0,500):
        gummi.goTo(rest, False)
        r.sleep()
    gummi.setCollisionResponses(False, False, False, False, False)
    rospy.loginfo("GummiArm is live!")
    rospy.sleep(1)

    reflex.listenHit = True

    while not rospy.is_shutdown():
        if reflex.hit == True:
            rospy.loginfo("Reflex Motion!!")
            for i in range(0,200):
            #only change equilibrum velocity
               gummi.elbow.moveWith(reflex.elbowdeg, 0.5)
               gummi.wrist.moveWith(reflex.wristdeg, 0.5)
               gummi.shoulderRoll.moveWith(reflex.rolldeg, 0.5)
               gummi.shoulderPitch.moveWith(reflex.pitchdeg, 0.5)
               gummi.shoulderYaw.moveWith(reflex.yawdeg, 0.5)
               r.sleep()


            # rospy.sleep(4)
            #
            # for i in range (0,500):
            #     gummi.goTo(rest, False)
            #     r.sleep()


            rospy.sleep(1)
            reflex.hit = False
            reflex.listenHit = True

        elif gummi.teleop == 0 and gummi.velocity_control == 0:
            gummi.doUpdate()

        gummi.publishJointState()
        r.sleep()

if __name__ == '__main__':
    main(sys.argv)
