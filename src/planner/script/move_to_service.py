#!/usr/bin/env python
import rospy
import time
import motion
import numpy
import almath
import sys
from naoqi import ALProxy
from planner.srv import MoveTo
from planner.srv import MoveToResponse
motionProxy =0
postureProxy=0

def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


def moveToNao(req):
    X= req.x
    Y= req.y
    Theta=req.theta
    

    StiffnessOn(motionProxy)
    motionProxy.setWalkArmsEnabled(False, False)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

    motionProxy.post.moveTo(X, Y, Theta)

    motionProxy.waitUntilMoveIsFinished()

    endRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))
    robotMove = m.pose2DInverse(initRobotPosition)*endRobotPosition
    print "Robot Move :", robotMove

    print "Return from MoveTo" 
   
    return MoveToResponse()
    #return 1

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    
    rospy.init_node('move_to_server')
    # Service declaration
    s = rospy.Service('move_to_service', MoveTo, moveToNao)
    rospy.spin()
			
		
