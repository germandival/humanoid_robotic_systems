#!/usr/bin/env python
# Group IndiaNAO Jones - Humanoid Robotic Systems - Final project
# Date: 05.02.2018
# Author: Francisco Zurita
import rospy
import time
import almath
import sys
import math
import motion
import numpy as np
from naoqi import *
from nao_control_tutorial_3.srv import TrackBall
from nao_control_tutorial_3.srv import TrackBallResponse
from mazeLine.srv import *

motionProxy = 0
ballFound = 0

def ball_position(req):


    #########################################################################################################
    '''
    First things first, that is, we check if the ArUco that signalizes the platform is found. If so, the ArUco
    coordinates are calculated by subscribing to the "point_next_step" service, and the coordinates are returned. 
    '''
    arucoPos = rospy.ServiceProxy('point_next_step', next_step)
    resp=arucoPos.call(10)

    if resp.id == 10:
        return TrackBallResponse([resp.x,resp.y,resp.z])


    #########################################################################################################
    '''
    Now that the ArUco has not been found, we proceed to look for the ball as a colored shape.
    '''
    tracker = ALProxy("ALColorBlobDetection", robotIP, PORT)
    videoDevice = ALProxy("ALVideoDevice",robotIP, PORT);
    AL_kTopCamera = 0
    AL_kQVGA = 1            # 320x240
    AL_kBGRColorSpace = 13

    #########################################################################################################
    '''
    Nao receives a message with the color of the ball he has to look for. It can be either yellow, red or blue.
    This establishes the BGR code of the color he seeks, as well as threshold for detection. 
    The BGR codes for this colors were determined in an auxiliary code using trackbars and tuning them to the 
    color of these specific woolen balls, both in light and darkness conditions.
    '''

    '''
     void ALColorBlobDetectionProxy::setColor(int r, int g, int b, int colorThres)

    '''
    if req.color == 'yellow':
        tracker.setColor(255,255,0,20)
    if req.color == 'blue':
        tracker.setColor(0,0,255,25)
    if req.color == 'red':
        tracker.setColor(255,0,0,20)


    #########################################################################################################
    '''
    The properties of the color blob he should seek are determined here. 
    A threshold of 25 pixels at least to consider it found, and a radius of 6cm, as well as circular shape.
    '''

    '''
     void ALColorBlobDetectionProxy::setObjectProperties(int minSize, float span, std::string shape)

        There are two overloads of this function:

            ALColorBlobDetectionProxy::setObjectProperties
            ALColorBlobDetectionProxy::setObjectProperties without the shape argument.


    '''

    tracker.setObjectProperties(25,0.06,"Circle") #cualquier cosa, cambiaste aca de 0.10 a 0.06
    tracker.subscribe("bubly")

    #########################################################################################################
    '''
    In order to be more robust to light conditions, positioning of the camera, and instability of the blob, a method from the
    library allows to englobe the blob in a circle, idealizing its shape. This method returns the pixels where the center is located, as well as
    the radius of this circle, also in pixels.
    If this method were not used, the center of the blob should have been calculated "by hand", finding the contours. This would have led
    to more error, computational time and false positives. But since the ball has a circular shape, that fact is taken advantage of. 
    '''
    '''
     AL::ALValue ALColorBlobDetectionProxy::getCircle()

        Retrieve the circle surrounding the blob.
        Returns:    the outer circle of the found blob as a vector of 3 values: x, y, radius in relative coordinates in the image (i.e. between 0 and 1).

        For example in VGA (640x480), for a circle of 50 pixels in radius and a center located at (123, 71), this function would return [123/640, 71/480, 50/640].
    '''
    lists = tracker.getCircle() #returns bounding circle of the blob in the form of x coordinate, y coordinate and diameter
                                #if value return by getCircle is not none return lists, else return (-1,-1,-1)


    #########################################################################################################
    '''
    Of course, pixel information is not really useful to direct a robot. It is better if this information in logically translated into an (x,y) coordinates.
    The yaw and pitch angles of the head are calculated. This angles represent how the head should move to center the gaze in the specified pixels, in this case, 
    the center of the circle.
    To estimate "y", that is the distance to the side of the robot, we take advantage of knowing the radius of the ball. Using the ratio of the radius and the 
    number of pixels that encode the radius of the circle, the distance in pixels to the center of the circle is used to estimate the distance in centimeters.
    To estimate "x", that is the distance to the front of the robot, we use trigonometry. We form a triangle being the sides (x,y). The sides of this trinagle are
    related as tan(yaw) = x/y, therefore, knowing yaw and y, we estimate x.
    We return this coordinates.  

    '''
    # Convert Circle coordinates into angle to turn.

    R = 0.06 #Radius of the target

    

    if lists!=None:

        px = lists[0]
        py = lists[1] # Center of the circle

        captureDevice = videoDevice.subscribeCamera("test", AL_kTopCamera, AL_kQVGA, AL_kBGRColorSpace, 10)

        # The distance to the ball is estimated in order to generate a direction vector to walk towards the ball.
        # The yaw angle is calculated from the center of the visual image from NAO's top camera towards the center of the sphere surrounding the ball.
        # Since the number of pixels of this sphere is calculated in the third argument of lists, the y (y in NAO, x from the camera) component is calculated
        # relating the cm/pixel ratio of the spehere.
        # Finally, the y deviation and distance to the target are related through the tangent of the yaw angle.

        [yaw,pitch]=videoDevice.getAngularPositionFromImagePosition(AL_kTopCamera, [lists[0],lists[1]]) # Angular deviation to camera axis
        pos_y = (px - 0.5) * R / (lists[2] - 0.5)  # This is, to the side of the robot.
        if yaw != 0:
            distance=pos_y/math.tan(yaw) # compute distance based on tan(angle)=opposite/adjacent
        else:
            distance = pos_y/math.tan(0.01)

        pos_x = distance
        if distance < 0:
            pos_x = -distance # This is, ahead of the robot.



        return TrackBallResponse([pos_x,pos_y,-1])

        #########################################################################################################
        '''
        Should no ArUco or circle have been found, we return a [-1,-1,-1] vector informing that nothing has been found.
        '''
    else:
        return TrackBallResponse([-1,-1,-1])

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('track_ball_server')
    print ("Service initiated")
    s = rospy.Service('track_ball', TrackBall, ball_position)
    rospy.spin()

