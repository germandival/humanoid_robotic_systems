#!/usr/bin/env python
import rospy
import time
import motion
import numpy
import almath
import sys
from naoqi import *
from planner.srv import ChangePosture
from planner.srv import ChangePostureResponse
motionProxy =0
postureProxy=0
autoMoves   =0

def changePostureNAO(req):
    mode= req.desiredPosture

    for x in range(0 ,50):
        motionProxy.setStiffnesses("RHand", 1.0)

    motionProxy.post.setAngles("RHand", 0.1, 0.2)
    autoMoves.setExpressiveListeningEnabled(False)

    if(mode == 1):
        postureProxy.goToPosture("StandInit", 0.6)
        time.sleep(3)
        motionProxy.setAngles("HeadPitch",(30*almath.TO_RAD),0.1)
        time.sleep(1)
    elif(mode == 2):
        postureProxy.goToPosture("Sit", 1.0)
        time.sleep(1)
    elif(mode == 3):
        motionProxy.setAngles("HeadPitch",(req.angle*almath.TO_RAD),0.3)
        time.sleep(1)
    elif(mode == 4):
        motionProxy.setAngles("HeadYaw",(req.angle*almath.TO_RAD),0.3)
        time.sleep(1)
    else:
        motionProxy.setAngles("HeadPitch",(30*almath.TO_RAD),0.1)
        time.sleep(1)
    
    #time.sleep(0.1)
    #motionProxy.setStiffnesses("Head",1.0) #Head
    
    #motionProxy.setStiffnesses("Head",0.0) #Head
    print "Posture changed" 
    return ChangePostureResponse()
    #return 1

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    autoMoves = ALProxy("ALAutonomousMoves", robotIP, PORT)
    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e
    rospy.init_node('change_posture_server')
    # Service declaration
    s = rospy.Service('change_posture_service', ChangePosture, changePostureNAO)
    rospy.spin()
			
		
