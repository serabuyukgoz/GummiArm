#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from antagonist import Antagonist
from direct_drive import DirectDrive

class Gummi:

    def __init__(self):
        self.teleop = rospy.get_param("~teleop", 1)
        if self.teleop:
            rospy.logwarn("Expecting teleoperation ('teleop' parameter in gummi.yaml file): " + str(self.teleop) + ".")

        self.velocity_control = rospy.get_param("~velocity_control", 1)
        if self.velocity_control:
            rospy.logwarn("Expecting velocity control ('velocity_control' parameter in gummi.yaml file): " + str(self.velocity_control) + ".")

        self.pi = 3.1416
        self.initVariables()
        self.initJoints()
        self.initPublishers()
        self.initSubscribers()

    def initVariables(self):
        self.jointNames = ("shoulder_yaw", "shoulder_roll", "shoulder_pitch", "upperarm_roll", "elbow", "forearm_roll", "wrist_pitch", "hand_dof1")
        self.shoulderRollVel = None
        self.shoulderPitchVel = None
        self.shoulderYawVel = None
        self.upperarmRollVel = None
        self.elbowVel = None
        self.forearmRollVel = None
        self.wristVel = None
        self.handDOF1Vel = None
        self.shoulderRollCocont = 0.3
        self.shoulderPitchCocont = 0.3
        self.shoulderYawCocont = 0.3
        self.elbowCocont = 0.3
        self.wristCocont = 0.6

    def initJoints(self):
        self.shoulderYaw = Antagonist("shoulder_yaw")
        self.shoulderRoll = Antagonist("shoulder_roll")
        self.shoulderPitch = Antagonist("shoulder_pitch")
        self.upperarmRoll = DirectDrive("upperarm_roll", self.pi)
        self.elbow = Antagonist("elbow")
        self.forearmRoll = DirectDrive("forearm_roll", 1.75*self.pi)
        self.wrist = Antagonist("wrist")
        self.headYaw = DirectDrive("head_yaw", 1.5*self.pi)
        self.handDOF1 = DirectDrive("hand_dof1", 1.5*self.pi)

        self.lastPositions = dict(zip(self.jointNames, self.getJointAngles()))
        self.lastVelocities = dict(zip(self.jointNames, self.getJointVelocities()))

    def initPublishers(self):
        self.jointStatePub = rospy.Publisher("gummi/joint_states", JointState,  queue_size=10)

    def initSubscribers(self):
        rospy.Subscriber('gummi/joint_commands', JointState, self.cmdCallback)

    def cmdCallback(self, msg):
        if self.teleop == 1 or self.velocity_control == 1:
            self.setVelocity(msg)  # Names will be checked inside servoTo
            self.setCocontraction(msg.effort[0], msg.effort[1], msg.effort[2], msg.effort[4], msg.effort[6])
            self.doVelocityUpdate()
        else:
            self.setCocontraction(abs(msg.effort[0]), abs(msg.effort[1]), abs(msg.effort[2]), abs(msg.effort[4]), abs(msg.effort[6]))
            if (msg.effort[0] >= 0) and (msg.effort[1] >= 0) and (msg.effort[2] >= 0) and (msg.effort[4] >= 0) and (msg.effort[6] >= 0):
                self.servoTo(msg)  # Names will be checked inside servoTo
            else:
                self.passiveHold()

    def doUpdate(self):
        self.shoulderYaw.doUpdate()
        self.shoulderRoll.doUpdate()
        self.shoulderPitch.doUpdate()
        self.upperarmRoll.doUpdate()
        self.elbow.doUpdate()
        self.forearmRoll.doUpdate()
        self.wrist.doUpdate()
        self.handDOF1.doUpdate()

    def doVelocityUpdate(self):
        if self.shoulderYawCocont < 0:
            self.shoulderYaw.moveWith(self.shoulderYawVel, abs(self.shoulderYawCocont))
        else:
            self.shoulderYaw.servoWith(self.shoulderYawVel, self.shoulderYawCocont)
        if self.shoulderRollCocont < 0:
            self.shoulderRoll.moveWith(self.shoulderRollVel, abs(self.shoulderRollCocont))
        else:
            self.shoulderRoll.servoWith(self.shoulderRollVel, self.shoulderRollCocont)
        if self.shoulderPitchCocont < 0:
            self.shoulderPitch.moveWith(self.shoulderPitchVel, abs(self.shoulderPitchCocont))
        else:
            self.shoulderPitch.servoWith(self.shoulderPitchVel, self.shoulderPitchCocont)
        self.upperarmRoll.servoWith(self.upperarmRollVel)
        if self.elbowCocont < 0:
            self.elbow.moveWith(self.elbowVel, abs(self.elbowCocont))
        else:
            self.elbow.servoWith(self.elbowVel, self.elbowCocont)
        self.forearmRoll.servoWith(self.forearmRollVel)
        if self.wristCocont == -999:
            self.wrist.passiveHold(0.0)
        else:
            if self.wristCocont < 0:
                self.wrist.moveWith(self.wristVel, abs(self.wristCocont))
            else:
                self.wrist.servoWith(self.wristVel, self.wristCocont)
        self.handDOF1.servoWith(self.handDOF1Vel)

        self.publishJointState()

    def publishJointState(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.jointNames
        msg.position = self.getJointAngles()
        msg.velocity = self.getJointVelocities()
        msg.effort = [self.shoulderYawCocont,
                      self.shoulderRollCocont,
                      self.shoulderPitchCocont,
                      0.0,
                      self.elbowCocont,
                      0.0,
                      self.wristCocont,
                      0.0]
        self.jointStatePub.publish(msg)

        self.lastPositions = dict(zip(msg.name, msg.position))
        self.lastVelocities = dict(zip(msg.name, msg.velocity))

    def getJointAngles(self):
        angles = list()
        angles.append(self.shoulderYaw.getJointAngle())
        angles.append(self.shoulderRoll.getJointAngle())
        angles.append(self.shoulderPitch.getJointAngle())
        angles.append(self.upperarmRoll.getJointAngle())
        angles.append(self.elbow.getJointAngle())
        angles.append(self.forearmRoll.getJointAngle())
        angles.append(self.wrist.getJointAngle())
        angles.append(self.handDOF1.getJointAngle())
        return angles

    def getJointVelocities(self):
        velocities = list()
        velocities.append(self.shoulderYaw.getJointVelocity())
        velocities.append(self.shoulderRoll.getJointVelocity())
        velocities.append(self.shoulderPitch.getJointVelocity())
        velocities.append(self.upperarmRoll.getJointVelocity())
        velocities.append(self.elbow.getJointVelocity())
        velocities.append(self.forearmRoll.getJointVelocity())
        velocities.append(self.wrist.getJointVelocity())
        velocities.append(self.handDOF1.getJointVelocity())
        return velocities

    def setVelocity(self, msg):
        for joint_name, joint_velocity in zip(msg.name, msg.velocity):
            self.lastVelocities[joint_name] = joint_velocity

        self.shoulderYawVel = self.lastVelocities['shoulder_yaw']
        self.shoulderRollVel = self.lastVelocities['shoulder_roll']
        self.shoulderPitchVel = self.lastVelocities['shoulder_pitch']
        self.upperarmRollVel = self.lastVelocities['upperarm_roll']
        self.elbowVel = self.lastVelocities['elbow']
        self.forearmRollVel = self.lastVelocities['forearm_roll']
        self.wristVel = self.lastVelocities['wrist_pitch']
        self.handDOF1Vel = self.lastVelocities['hand_dof1']

    def setCocontraction(self, shoulderYaw, shoulderRoll, shoulderPitch, elbow, wrist):
        self.shoulderYawCocont = shoulderYaw
        self.shoulderRollCocont = shoulderRoll
        self.shoulderPitchCocont = shoulderPitch
        self.elbowCocont = elbow
        self.wristCocont = wrist

    def servoTo(self, msg):
        if self.teleop == 0:

            for joint_name, joint_angle in zip(msg.name, msg.position):
                self.lastPositions[joint_name] = joint_angle

            self.shoulderYaw.servoTo(self.lastPositions['shoulder_yaw'], self.shoulderYawCocont)
            self.shoulderRoll.servoTo(self.lastPositions['shoulder_roll'], self.shoulderRollCocont)
            self.shoulderPitch.servoTo(self.lastPositions['shoulder_pitch'], self.shoulderPitchCocont)
            self.upperarmRoll.servoTo(self.lastPositions['upperarm_roll'])
            self.elbow.servoTo(self.lastPositions['elbow'], self.elbowCocont)
            self.forearmRoll.servoTo(self.lastPositions['forearm_roll'])
            self.wrist.servoTo(self.lastPositions['wrist_pitch'], self.wristCocont)
            self.handDOF1.servoTo(self.lastPositions['hand_dof1'])


            self.publishJointState()
        else:
            rospy.logwarn("Asked to servo to pose, but ignoring as in teleop mode. Check gummi.yaml file.")

    def goTo(self, positions, now):
        if self.teleop == 0:
            self.shoulderYaw.goTo(positions[0], self.shoulderYawCocont, now)
            self.shoulderRoll.goTo(positions[1], self.shoulderRollCocont, now)
            self.shoulderPitch.goTo(positions[2], self.shoulderPitchCocont, now)
            self.upperarmRoll.servoTo(positions[3])
            self.elbow.goTo(positions[4], self.elbowCocont, now)
            self.forearmRoll.servoTo(positions[5])
            self.wrist.goTo(positions[6], self.wristCocont, now)
            self.handDOF1.servoTo(positions[7])
            self.publishJointState()
        else:
            rospy.logwarn("Asked to go to pose, but ignoring as in teleop mode. Check gummi.yaml file.")

    def goRestingPose(self, now):
        self.shoulderYaw.goTo(0, self.shoulderYawCocont, now)
        self.shoulderRoll.goTo(0, self.shoulderRollCocont, now)
        self.shoulderPitch.goTo(0, self.shoulderPitchCocont, now)
        self.upperarmRoll.servoTo(0)
        self.elbow.goTo(0, self.elbowCocont, now)
        self.forearmRoll.servoTo(0)
        self.wrist.goTo(0, self.wristCocont, now)
        self.handDOF1.servoTo(0)

    def passiveHold(self):
        self.shoulderYaw.passiveHold(self.shoulderYawCocont)
        self.shoulderRoll.passiveHold(self.shoulderRollCocont)
        self.shoulderPitch.passiveHold(self.shoulderPitchCocont)
        self.upperarmRoll.doUpdate()
        self.elbow.passiveHold(self.elbowCocont)
        self.forearmRoll.doUpdate()
        self.wrist.passiveHold(self.wristCocont)
        self.handDOF1.doUpdate()

    def doGradualStartup(self):
        self.shoulderYaw.moveTo(-0.05, self.shoulderYawCocont)
        rospy.sleep(1)
        self.shoulderRoll.moveTo(-0.25, self.shoulderRollCocont)
        rospy.sleep(1)
        self.shoulderPitch.moveTo(0.25, self.shoulderPitchCocont)
        rospy.sleep(1)
        self.upperarmRoll.servoTo(0)
        rospy.sleep(1)
        self.elbow.moveTo(-0.6, self.elbowCocont)
        rospy.sleep(1)
        self.forearmRoll.servoTo(0)
        rospy.sleep(1)
        self.wrist.moveTo(0, self.wristCocont)
        rospy.sleep(1)
        self.handDOF1.servoTo(-2.2)
        rospy.sleep(1)
        self.headYaw.servoTo(0)
        rospy.sleep(1)

    def setCollisionResponses(self, shoulderYaw, shoulderRoll, shoulderPitch, elbow, wrist):
        self.shoulderYaw.setCollisionResponse(shoulderYaw)
        self.shoulderRoll.setCollisionResponse(shoulderRoll)
        self.shoulderPitch.setCollisionResponse(shoulderPitch)
        self.elbow.setCollisionResponse(elbow)
        self.wrist.setCollisionResponse(wrist)

    def setReflexFlag(self):
        items = list()
        items.append(self.shoulderYaw.is_moving())
        items.append(self.shoulderRoll.is_moving())
        items.append(self.shoulderPitch.is_moving())
        items.append(self.elbow.is_moving())
        items.append(self.wrist.is_moving())
	    # print(items)
        if any(items):
            self.shoulderYaw.doReflex = False
            self.shoulderRoll.doReflex = False
            self.shoulderPitch.doReflex = False
            self.elbow.doReflex = False
            self.wrist.doReflex = False
        else:
            self.shoulderYaw.doReflex = True
            self.shoulderRoll.doReflex = True
            self.shoulderPitch.doReflex = True
            self.elbow.doReflex = True
            self.wrist.doReflex = True
