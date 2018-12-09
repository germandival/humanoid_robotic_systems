#!/usr/bin/env python
import rospy
import time
import motion
import numpy as np
import almath
import sys
from naoqi import ALProxy
from planner.srv import PosArucoFloor
from planner.srv import PosArucoFloorResponse
motionProxy =0
postureProxy=0

def calculatePosArucoFloor(req):
    #mode= req.desiredPosture

    
    print "Calculate Aruco Pos"
    posexyz=np.zeros((4,1))#auxiliar vector to allocate the original position and to transform them from the input coordinates to the coordinates that are sendet to the robot. transformed to torso TF

    posexyz[0, 0]=req.x
    posexyz[1, 0]=req.y
    posexyz[2, 0]=req.z
    posexyz[3, 0]=1 


    chainName = "CameraBottom"#set the chain that will be moved
    space = motion.SPACE_TORSO
    tTCSensor = motionProxy.getTransform(chainName, space, True)#transformation matrix between the TF of the original TF position and the desired TF position
    T = np.matrix([tTCSensor[0:4],tTCSensor[4:8],tTCSensor[8:12],tTCSensor[12:]])#set the transformation matrix in numpy matrix to perform operations
    H=np.array([[0,0,1,0],
                [-1,0,0,0],
                [0,-1,0,0],
                [0,0,0,1]], np.int32)# rotaion matrix to transofm the TF in bottom camara to camera frame. the AruCo markers position is given in bottom camera frame. With the motionProxy.getTransform(chainName, space, True) line of code we can get the transformation between camera frame and torso TF. for that reason we need to get the position of the ArUco markers in camera frame.

    posexyz=np.dot(H,posexyz)#get the position in cammera frame
    posexyz=np.dot(T,posexyz)#get the position in torso

    
   # res.x=posexyz[0,0]
    #res.y=posexyz[1,0]
    #res.z=posexyz[2,0]

    print posexyz[0,0]
    print posexyz[1,0]
    print posexyz[2,0]
    #posexyz[0/1/2,0]

    vector6D=[]
    for x in range(1):
        row = []
        for y in range(6):
            row.append(0)
        vector6D.append(row)

    for j in range (0,3):
        vector6D[0][j]=posexyz[j,0]

    print "Calculated"

    return PosArucoFloorResponse(posexyz)


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e
    rospy.init_node('pos_aruco_floor_server')
    # Service declaration
    s = rospy.Service('pos_aruco_floor_service', PosArucoFloor, calculatePosArucoFloor)
    rospy.spin()
			
		
