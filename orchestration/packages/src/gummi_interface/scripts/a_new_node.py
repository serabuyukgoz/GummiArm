#!/usr/bin/env python

import rospy
import sys
import random
import time

from dynamixel_msgs.msg import JointState
from std_msgs.msg import Int8

is_moving = {"shoulder_abductor_controller": 0,
             "shoulder_adductor_controller": 0,
             "shoulder_extensor_controller": 0,
             "shoulder_flexor_controller": 0,
             "triceps_controller": 0,
             "biceps_controller": 0,
             "horizontal_abductor_controller": 0,
             "horizontal_adductor_controller": 0,
             "wrist_extensor_controller": 0,
             "wrist_flexor_controller": 0}

error_thres = 0.05

def Callback(msg):
    is_moving[msg.name] = msg.error


def main(args):

    rospy.init_node('GummiMoving', anonymous=False)
    r = rospy.Rate(60)

    rospy.Subscriber("/shoulder_abductor_controller/state", JointState, Callback)
    rospy.Subscriber("/shoulder_adductor_controller/state", JointState, Callback)

    rospy.Subscriber("/shoulder_extensor_controller/state", JointState, Callback)
    rospy.Subscriber("/shoulder_flexor_controller/state", JointState, Callback)

    rospy.Subscriber("/triceps_controller/state", JointState, Callback)
    rospy.Subscriber("/biceps_controller/state", JointState, Callback)

    rospy.Subscriber("/horizontal_abductor_controller/state", JointState, Callback)
    rospy.Subscriber("/horizontal_adductor_controller/state", JointState, Callback)

    rospy.Subscriber("/wrist_extensor_controller/state", JointState, Callback)
    rospy.Subscriber("/wrist_flexor_controller/state", JointState, Callback)

    moving = rospy.Publisher('is_arm_moving', Int8 , queue_size=1)

    while not rospy.is_shutdown():
        if(any([abs(li)>error_thres for li in is_moving.values()])):
            moving.publish(1)
        else:
            moving.publish(0)
        r.sleep()

if __name__ == '__main__':
    main(sys.argv)
