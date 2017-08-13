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
        rospy.init_node('gummi_moveit_config', anonymous=True)
        self.r = rospy.Rate(60) #not sure needed or not

        rospy.Subscriber('/hit_signal', HitDetectReflex, self.Callback)

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
        self.path(hit, name, amplitude)

    def path(self, hit, name, amp):
        
        if hit > 0:
            print("UP")
            if name == 'elbow' or name == 'shoulder_roll':
                if amp > 0.3:
                    print("left top")
                    angles = [0.5062136600022616, 0.9612946270750019, -0.08692557798018634, -0.7286408742456796, 0.0, -0.010226538585904275, 0.0]
                    self.move(angles)
                else:
                    print("left mid")
                    angles = [0.5931392379824479, 0.0, 0.0, -0.8513593372765309, 0.0, -0.010226538585904275, 0.0]
                    self.move(angles)
            else:
                print("left bottom")
                angles = [0.29145634969827183, -0.07158577010132992, 0.0, -0.4724660826687775, -0.8385761640441506, -0.010226538585904275, 0.10737865515199489] 
                self.move(angles)
        elif hit < 0:
            if name == 'elbow' or name == 'shoulder_roll':
                print("right back")
                angles = [-0.41417481272912315, -0.4908738521234052, 0.4192880820220753, 1.2701360923693108, -0.8232363561652941, -0.010226538585904275, 0.3119094268700804] 
                self.move(angles)
            else:
                if amp > 0.3:
                    print("right bottom")
                    angles = [-0.260776733940559, -0.19430423313218123, 0.8385761640441506, -0.20708740636456155, -0.7311975088921556, -0.010226538585904275, 0.24032365676875048]
                    self.move(angles)
                else:
                    print("mid bottom")
                    angles = [-0.1227184630308513, -0.07158577010132992, 0.0, 0.15800002115222103, -0.782330201821677, -0.010226538585904275, 0.06647250080837779] 
                    self.move(angles)

    def move(self, angles): 
       # print "============ Joint values: ", group_variable_values  
        print "Move___"   
        group_variable_values = self.group.get_current_joint_values()
        #print "============ Joint values: ", group_variable_values
        group_variable_values = angles
        self.group.set_joint_value_target(group_variable_values)

        plan2 = self.group.plan()

        self.group.go(wait=True)

        self.group.clear_pose_targets()

        group_variable_values = self.group.get_current_joint_values()
       # print "============ Joint values: ", group_variable_values
    
def main(args):
    print("start")
    ref = StaticReflexMotion()

    while not rospy.is_shutdown():
        ref.r.sleep()

if __name__ == '__main__':
    main(sys.argv)
