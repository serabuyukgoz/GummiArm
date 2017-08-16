#!/usr/bin/env python

import rospy
import sys
import random
import time
import numpy as np

from gummi_interface.msg import HitDetectReflex

from dynamixel_msgs.msg import JointState
from std_msgs.msg import Int8

reflex_joint = {"shoulder_pitch_encoder_controller": 0,
                "shoulder_roll_encoder_controller": 0,
                "shoulder_yaw_encoder_controller": 0,
                "elbow_encoder_controller": 0,
                "wrist_encoder_controller": 0 }

error_thres = 0.15

window_size = 10
moving_avg_flag = [0]*window_size

def hitFlagCall(msg):
    global hit_flag
    if msg.hit == False:
        print("hitFlagCall")
        hit_flag == False    

def Callback(msg):
    global moving_flag
    global hit_flag

    if sum(moving_avg_flag)<10:#moving_flag == False:
        print("moving_flag == False")
        return

    if abs(msg.velocity) > 0.15 and hit_flag == False: 
        
        hit_flag = True
        rospy.loginfo(msg.velocity)
        findAmplitude(msg.velocity, msg.name)     
        
def StopCall(data):
    global moving_flag
    if data.data == 1:
        #moving_flag = False
        moving_avg_flag.append(0)
        moving_avg_flag.pop()
    else:
        #moving_flag = True 
        moving_avg_flag.append(1)
        moving_avg_flag.pop()


def findAmplitude(vel, name):        
    amplitude = abs(vel)
    direction = np.sign(vel)
    magnitude = amplitude*direction 
    rospy.logwarn(str(magnitude) + str(name))
    hitPublish(amplitude,magnitude, direction, name)

def hitPublish(amplitude,magnitude, direction , name):
    global pubHit
    hitmsg = HitDetectReflex()
    hitmsg.name = name
    hitmsg.hit = True
    hitmsg.amplitude = amplitude
    hitmsg.direction = direction
    hitmsg.magnitude = magnitude
   # hitmsg.time = reflexTime
    pubHit.publish(hitmsg)     

def main(args):
    global moving_flag
    global hit_flag
    global pubHit

    moving_flag = False
    hit_flag = False
    rospy.init_node('GummiReflex', anonymous=False)
    r = rospy.Rate(60)

    rospy.Subscriber("is_arm_moving", Int8, StopCall)

    rospy.Subscriber('hit_signal', HitDetectReflex, hitFlagCall)

    rospy.Subscriber("/shoulder_pitch_encoder_controller/state", JointState, Callback)
    rospy.Subscriber("/shoulder_roll_encoder_controller/state", JointState, Callback)
    rospy.Subscriber("/shoulder_yaw_encoder_controller/state", JointState, Callback)
    rospy.Subscriber("/elbow_encoder_controller/state", JointState, Callback)
    rospy.Subscriber("/wrist_encoder_controller/state", JointState, Callback)          
    
    #hit = rospy.Publisher('hit', Int8, queue_size=1, latched=True)
    pubHit = rospy.Publisher('hit_signal', HitDetectReflex, queue_size=1, latch=True) 


    while not rospy.is_shutdown():
       
        r.sleep()
if __name__ == '__main__':
    main(sys.argv)
