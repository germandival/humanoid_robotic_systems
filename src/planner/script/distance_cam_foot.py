#!/usr/bin/env python
import rospy
import time
import motion
import numpy
import almath
import sys
from naoqi import *
from planner.srv import DistanceCamFoot
from planner.srv import DistanceCamFootResponse
motionProxy =0
postureProxy=0

from tf import TransformListener
from tf import transformations

def measureDistanceNAO(req):
    AL_kTopCamera = 1
    x=req.x
    y=req.y
    videoDevice = ALProxy("ALVideoDevice",robotIP, PORT);
    [yaw,pitch]=videoDevice.getAngularPositionFromImagePosition(AL_kTopCamera, [x,y]) # Angular deviation to camera axis
      
    print("x en service: ")
    print(x) 
    print("y en service: ")
    print(y) 
    print("Yaw en service :") 
    print yaw


    useSensorValues = True # Enables the use of sensol values to determine the positions
    frame=motion.FRAME_ROBOT  #sets the torso as the final frame to express the positions

   
    name="LFoot"
    #current          = motionProxy.getPosition("CameraBottom", frame, useSensorValues) # get the current position
    

   

    

    return DistanceCamFootResponse(yaw)


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    
    rospy.init_node('distance_cam_foot')
    # Service declaration
    s = rospy.Service('distance_cam_foot_service', DistanceCamFoot, measureDistanceNAO)
    rospy.spin()
			
		
