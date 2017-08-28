#!/usr/bin/env python

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from gummi_interface.msg import HitDetectReflex

class StaticReflexMotion:
    def __init__(self):
        print("Initials")
        rospy.init_node('sera', anonymous=False)
        self.r = rospy.Rate(60) #not sre needed or not

        rospy.Subscriber('/hit_signal', HitDetectReflex, self.Callback)
        self.pubHit = rospy.Publisher('/hit_signal', HitDetectReflex, queue_size=1, latch=True)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_arm")

        display_trajectory_publisher = rospy.Publisher(
                                    '/moveit/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)


    def Callback(self, data):
        name = data.name
        hit = data.direction
        amplitude = data.amplitude
        print(name + " Data " + str(hit))
        if hit != 0:
            self.path(hit, name, amplitude)

    def path(self, hit, name, amp):

        waypoints = []
        waypoints.append(self.group.get_current_pose().pose)
        y = waypoints[0].position.y
        z = waypoints[0].position.z
        x = waypoints[0].position.x
        if hit < 0:
            print("UP")
            if name == 'elbow' or name == 'shoulder_roll':
                if amp > 0.15:
                    print("left top")
                    angles = [0.5062136600022616, 0.9612946270750019, -0.08692557798018634, -0.7286408742456796, 0.0, -0.010226538585904275, 0.0]
                    x = 0.26
                    y = 0.28
                    z = 0.15
                    xo = -0.518642435076
                    yo = -0.698823016697
                    zo = 0.0458006177463
                    wo = 0.490467857548
                    self.moveCartesion(x,y,z, xo, yo, zo, wo)
                    #self.move(angles)
                else:
                    print("left mid")
                    angles = [0.5931392379824479, 0.0, 0.0, -0.8513593372765309, 0.0, -0.010226538585904275, 0.0]
                    x = 0.11
                    y = 0.23
                    z = -0.18
                    xo = -0.53326345939
                    yo = -0.46982618249
                    zo = 0.465060085338
                    wo =  0.527837624797
                    self.moveCartesion(x,y,z, xo, yo, zo, wo)
                   # self.move(angles)
            else:
                print("left bottom")
                angles = [0.29145634969827183, -0.07158577010132992, 0.0, -0.4724660826687775, -0.8385761640441506, -0.010226538585904275, 0.10737865515199489]
                x = 0.17
                y = 0.03
                z = -0.43
                xo = -0.697746286537
                yo = 0.00653165155508
                zo = 0.506503030009
                wo = 0.50651963214
                self.moveCartesion(x,y,z, xo, yo, zo, wo)
                #self.move(angles)
        elif hit > 0:
            if name == 'elbow' or name == 'shoulder_roll':
                print("right back")
                angles = [-0.41417481272912315, -0.4908738521234052, 0.4192880820220753, 1.2701360923693108, -0.8232363561652941, -0.010226538585904275, 0.3119094268700804]
                x = -0.31
                y = -0.47
                z = -0.25
                xo = -0.514313492806
                yo = 0.717868654045
                zo = -0.298103535486
                wo = 0.362326522336
                self.moveCartesion(x,y,z, xo, yo, zo, wo)
               # self.move(angles)
            else:
                if amp > 0.15:
                    print("right bottom")
                    angles = [-0.260776733940559, -0.19430423313218123, 0.8385761640441506, -0.20708740636456155, -0.7311975088921556, -0.010226538585904275, 0.24032365676875048]
                    #self.move(angles)
                    x = 0.08
                    y = -0.49
                    z = -0.30
                    xo = -0.861574432111
                    yo = 0.312785852249
                    zo = 0.237735528399
                    wo = 0.321459681922
                    self.moveCartesion(x,y,z, xo, yo, zo, wo)
                else:
                    print("mid bottom")
                    angles = [-0.1227184630308513, -0.07158577010132992, 0.0, 0.15800002115222103, -0.782330201821677, -0.010226538585904275, 0.06647250080837779]
                    #self.move(angles)
                    x = 0.18
                    y = -0.19
                    z = -0.43
                    xo = -0.615683815783
                    yo = 0.360449964398
                    zo = 0.178129112922
                    wo = 0.677701469142
                    self.moveCartesion(x,y,z, xo, yo, zo, wo)

    def move(self, angles):
        print "Move___"
        group_variable_values = self.group.get_current_joint_values()
        print "============  ", self.group.get_current_joint_values()
        group_variable_values = angles
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(group_variable_values)

        plan2 = self.group.plan()

        self.group.go(wait=True)

        self.group.clear_pose_targets()

        group_variable_values = self.group.get_current_joint_values()
       # print "============ Joint values: ", group_variable_values
        #self.is_running = False

    def moveCartesion(self, x,y,z, xo, yo, zo, wo):
        print "Pose=>",self.group.get_current_pose().pose
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x =  xo
        pose_target.orientation.y = yo
        pose_target.orientation.z = zo
        pose_target.orientation.w = wo
        self.group.set_pose_target(pose_target)
        print "Pose Set =>",pose_target
        plan1 = self.group.plan()

        self.group.go(wait=True)

        self.group.clear_pose_targets()

        self.hitPublish()

    def hitPublish(self):
        hitmsg = HitDetectReflex()
        print("return published")
        hitmsg.hit = False
        self.pubHit.publish(hitmsg)

def main(args):
    print("start")
    ref = StaticReflexMotion()

    while not rospy.is_shutdown():
        ref.r.sleep()

if __name__ == '__main__':
    main(sys.argv)
