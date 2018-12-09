#!/usr/bin/env python
# Group IndiaNAO Jones - Humanoid Robotic Systems - Final project
# Date: 05.02.2018
# Author: Francisco Zurita
import rospy
import time
import almath
import sys
import motion
import numpy
import time
from naoqi import ALProxy
from nao_control_tutorial_3.srv import HandOverBall
from nao_control_tutorial_3.srv import HandOverBallResponse

motionProxy = 0;

def handover(req):

    Larm = "LArm"
    Rarm = "RArm"
    frame = motion.FRAME_TORSO
    useSensorValues = False
    fractionMaxSpeed = 0.01
    axisMask = 7
    axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL]

   
    # Method 1. With one hand.

    #########################################################################################################
    '''
    This service simply lifts the arm with the target and after two seconds, releases the ball. 
    Angles are interpolated instead of positions to have a more specific control of the limb's movements.
    '''
    #########################################################################################################

    effectorList = []
    pathList     = []


    #left_shoulder_pitch = -50
    right_shoulder_pitch = -60
    #left_shoulder_roll = -15
    right_shoulder_roll = 15
    right_elbow_yaw  =  90
    right_wrist_yaw  = -80
    #left_elbow_roll = -15
    right_elbow_roll = 45

    #names = ["LShoulderPitch", "RShoulderPitch", "LShoulderRoll", "RShoulderRoll", "LElbowRoll", "RElbowRoll"]
    names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
    angleLists = [right_shoulder_pitch*almath.TO_RAD, 
                  right_shoulder_roll*almath.TO_RAD, 
                  right_elbow_yaw*almath.TO_RAD, 
                  right_elbow_roll*almath.TO_RAD, 
                  right_wrist_yaw*almath.TO_RAD]
    timeLists  = [4.0, 4.0, 4.0, 4.0, 4.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)


    time.sleep(1)

    #motionProxy.openHand("LHand")
    time.sleep(2) #TIME FOR THE HUMAN TO GRAB THE BALL
    motionProxy.post.setAngles("RHand", 0.9, 0.2)
    motionProxy.post.openHand("RHand")

    '''
    # Method 2. Both hands.

    #########################################################################################################
    Method that uses both hands to hand over the ball. Was not completly debugged but provides better
    dynamics and a more pleasant motion.
    #########################################################################################################
    

    motionProxy.post.setAngles("RHand", 0.0, 0.1)

    effectorList = []
    pathList     = []
    
    timeList     = [[6.0], [4.0]]
    
    target_larm = [0.10,   0.0, 0.00, 0, 0, 0]
    target_rarm = [0.10, - 0.0, 0.10 , 0, 0, 0]

    effectorList.append("LArm")
    targetPos = almath.Position6D(target_larm)
    pathList.append(list(targetPos.toVector()))

    effectorList.append("RArm")
    targetPos = almath.Position6D(target_rarm)
    pathList.append(list(targetPos.toVector()))


    motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)


    

    # Put wrists and elbows in the correct position

    left_ang_wrist_rad= -90
    right_ang_wrist_rad= 0
    left_ang_elbow_rad= -90
    right_ang_elbow_rad= 0

    #names      = ["LWristYaw", "RWristYaw", "LElbowYaw", "RElbowYaw"]
    names      = ["LWristYaw", "RWristYaw", "LElbowYaw", "RElbowYaw"]
    #angleLists = [left_ang_wrist_rad*almath.TO_RAD, right_ang_wrist_rad*almath.TO_RAD, left_ang_elbow_rad*almath.TO_RAD, right_ang_elbow_rad*almath.TO_RAD]
    angleLists = [left_ang_wrist_rad*almath.TO_RAD, right_ang_wrist_rad*almath.TO_RAD, left_ang_elbow_rad*almath.TO_RAD, right_ang_elbow_rad*almath.TO_RAD]
    #timeLists  = [2.0, 2.0, 2.0, 2.0]
    timeLists  = [3.0, 3.0, 3.0, 3.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    

    time.sleep(4)

    # Now approach left hand upwards.

    motionProxy.post.openHand("LHand")

    effectorList = []
    pathList     = []

    target_larm = [0.10, 0.0, 0.12, 0, 0, 0]
    effectorList.append("LArm")
    targetPos = almath.Position6D(target_larm)
    pathList.append(list(targetPos.toVector()))
    timeList    = [[3.0]]

    motionProxy.positionInterpolations(effectorList, frame, pathList, axisMask, timeList)

    time.sleep(3)

    # Finally open the hand slowly and rise the arms.


    left_ang_wrist_rad= -60
    right_ang_wrist_rad= 60
    left_ang_elbow_rad= -90
    right_ang_elbow_rad= 90

    #names      = ["LWristYaw", "RWristYaw", "LElbowYaw", "RElbowYaw"]
    names      = ["LWristYaw", "RWristYaw", "LElbowYaw", "RElbowYaw"]
    #angleLists = [left_ang_wrist_rad*almath.TO_RAD, right_ang_wrist_rad*almath.TO_RAD, left_ang_elbow_rad*almath.TO_RAD, right_ang_elbow_rad*almath.TO_RAD]
    angleLists = [left_ang_wrist_rad*almath.TO_RAD, right_ang_wrist_rad*almath.TO_RAD, left_ang_elbow_rad*almath.TO_RAD, right_ang_elbow_rad*almath.TO_RAD]
    #timeLists  = [2.0, 2.0, 2.0, 2.0]
    timeLists  = [4.0, 4.0, 4.0, 4.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    

    effectorList = []
    pathList     = []
    
    timeList     = [[4.0], [4.0]]
    
    target_larm = [0.15,   0.03, 0.12, 0, 0, 0]
    target_rarm = [0.15, - 0.03, 0.12, 0, 0, 0]

    effectorList.append("LArm")
    targetPos = almath.Position6D(target_larm)
    pathList.append(list(targetPos.toVector()))

    effectorList.append("RArm")
    targetPos = almath.Position6D(target_rarm)
    pathList.append(list(targetPos.toVector()))


    motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
    
    time.sleep(5)

    motionProxy.post.setAngles("RHand", 1.0, 0.1)


    effectorList = []
    pathList     = []
    '''


    return ("Returned")



if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('handover_ball_server')
    print ("Service initiated")
    s = rospy.Service('handover_ball', HandOverBall, handover)
    rospy.spin()

