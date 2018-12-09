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
from nao_control_tutorial_3.srv import GrabBall
from nao_control_tutorial_3.srv import GrabBallResponse

motionProxy = 0;

def ball_location(req):

    
    '''
	The service receives the x,y,z coordinates of the object it has to grab.
	The grabbing process is done through a series of steps, provided the object is in front of the robot.
	'''

    posX=req.pos_ini[0]
    posY=req.pos_ini[1]
    posZ=req.pos_ini[2]

    effectorList = []
    pathList     = []

    #########################################################################################################
    # Step 1: Open arms wide so you don't hit the box.
    # Since a big box is the platform, NAO first opens his arms wide to avoid getting stuck with it.
    #########################################################################################################

    Larm = "LArm"
    Rarm = "RArm"
    frame = motion.FRAME_TORSO
    useSensorValues = False
    fractionMaxSpeed = 0.01
    axisMask = 7
    axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL]
    timeList     = [[2.0], [2.0]]
    
    target_larm = [0.05,   0.25, 0, 0, 0, 0]
    target_rarm = [0.05, - 0.25, 0, 0, 0, 0]
    
    effectorList.append("LArm")
    targetPos = almath.Position6D(target_larm)
    pathList.append(list(targetPos.toVector()))

    effectorList.append("RArm")
    targetPos = almath.Position6D(target_rarm)
    pathList.append(list(targetPos.toVector()))


    motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
    time.sleep(2)

    #########################################################################################################
    # Step 2: Put LArm and RArm effectors on both sides of the ball.
    # Initial movement, to center the target slowly and put it between hands, in case the robot
    # is not exactly in front of the target.
    # A loop is started here, since this process is going to be repeated up to 5 times, in case the object was 
    # not grabbed. If the object is not grabbed after three tries, it is abandoned, since it most likely fell from
    # the platform.
    #########################################################################################################

    for i in range(5):

        effectorList = []
        pathList     = []

        Larm = "LArm"
        Rarm = "RArm"
        frame = motion.FRAME_TORSO
        useSensorValues = False
        fractionMaxSpeed = 0.01
        axisMask = 7
        axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL]
        timeList     = [[3.0], [3.0]]
       
        target_larm = [posX, posY + 0.15, posZ, 0, 0, 0]
        target_rarm = [posX, posY - 0.15, posZ, 0, 0, 0]

        effectorList.append("LArm")
        targetPos = almath.Position6D(target_larm)
        pathList.append(list(targetPos.toVector()))

        effectorList.append("RArm")
        targetPos = almath.Position6D(target_rarm)
        pathList.append(list(targetPos.toVector()))


        motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
        time.sleep(3)

        #########################################################################################################
        # 3 Step 3: Turn wrists in a proper grasping angle.
        #########################################################################################################

        left_ang_wrist_rad= -10
        right_ang_wrist_rad= 10

        #names      = ["LWristYaw", "RWristYaw", "LElbowYaw", "RElbowYaw"]
        names      = ["LWristYaw", "RWristYaw"]
        #angleLists = [left_ang_wrist_rad*almath.TO_RAD, right_ang_wrist_rad*almath.TO_RAD, left_ang_elbow_rad*almath.TO_RAD, right_ang_elbow_rad*almath.TO_RAD]
        angleLists = [left_ang_wrist_rad*almath.TO_RAD, right_ang_wrist_rad*almath.TO_RAD]
        #timeLists  = [2.0, 2.0, 2.0, 2.0]
        timeLists  = [1.0, 1.0]
        isAbsolute = True
        motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        time.sleep(1)

        #########################################################################################################
        # 4 Now open your hands...
        #########################################################################################################

        motionProxy.openHand("LHand")
        motionProxy.openHand("RHand")

        #########################################################################################################
        # 5 Step 5 NAO squeezes the target and closes his right hand. And keeps it closed.
        #########################################################################################################

        fractionMaxSpeed = 0.1

        effectorList = []
        pathList     = []
        dy = 0.11
        timeLists  = [2.0, 2.0]


        effectorList.append("LArm")
        currentPos = motionProxy.getPosition("LArm", frame, useSensorValues)
        targetPos = almath.Position6D(target_larm)
        targetPos.y -= dy
        pathList.append(list(targetPos.toVector()))


        effectorList.append("RArm")
        currentPos = motionProxy.getPosition("RArm", frame, useSensorValues)
        targetPos = almath.Position6D(target_rarm)
        targetPos.y += dy
        pathList.append(list(targetPos.toVector()))


        motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
        time.sleep(2)

        #########################################################################################################
        # 6 Close your hand...
        #########################################################################################################

        for x in range(0 ,50):
            motionProxy.setStiffnesses("RHand", 1.0)

        motionProxy.post.setAngles("RHand", 0.0, 0.1)
        time.sleep(2)

        #########################################################################################################
        # 7 Check if hand is closed
        #########################################################################################################
        sensedAngles = motionProxy.getAngles("RHand", True) #Checks the sensed angles.

        if sensedAngles > 0.1: 
            print ("Ball grabbed successfully.")
            break
        else:
            print ("Ball grabbed UNsuccessfully")


    #########################################################################################################
    # Step 8: Now with the object in hand, NAO slowly attempts to put his arms to the side of his body
    # slowly, without dropping the object or hitting the box. After that, control is returned to the planner.
    #########################################################################################################


    effectorList = []
    pathList     = []
    timeList     = [[4.0], [4.0]]
    
    target_larm = [0.00,   0.25, 0.05, 0, 0, 0]
    target_rarm = [0.00, - 0.25, 0.05, 0, 0, 0]

    effectorList.append("LArm")
    targetPos = almath.Position6D(target_larm)
    pathList.append(list(targetPos.toVector()))

    effectorList.append("RArm")
    targetPos = almath.Position6D(target_rarm)
    pathList.append(list(targetPos.toVector()))


    motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
    time.sleep(1)


    effectorList = []
    pathList     = []
    timeLists  	 = [3.0, 3.0]

    target_larm = [ 0.00,   0.12, -0.10, 0, 0, 0]
    target_rarm = [ 0.00, - 0.12, -0.10, 0, 0, 0]

    effectorList.append("LArm")
    targetPos = almath.Position6D(target_larm)
    pathList.append(list(targetPos.toVector()))

    effectorList.append("RArm")
    targetPos = almath.Position6D(target_rarm)
    pathList.append(list(targetPos.toVector()))


    motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
    time.sleep(1)


    '''
    #names      = ["LWristYaw", "RWristYaw", "LElbowYaw", "RElbowYaw"]
    names      = ["RElbowYaw", "RWristYaw"]
    #angleLists = [left_ang_wrist_rad*almath.TO_RAD, right_ang_wrist_rad*almath.TO_RAD, left_ang_elbow_rad*almath.TO_RAD, right_ang_elbow_rad*almath.TO_RAD]
    angleLists = [-45*almath.TO_RAD, -45*almath.TO_RAD]
    #timeLists  = [2.0, 2.0, 2.0, 2.0]
    timeLists  = [2.0, 2.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1)
    '''

    return GrabBallResponse(req.pos_ini)




if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('grab_ball_server')
    print ("Service initiated")
    s = rospy.Service('grab_ball', GrabBall, ball_location)
    rospy.spin()

