/* Group IndiaNAO Jones - Humanoid Robotic Systems - Final project
   Date: 05.02.2018
   Author: Daniela Sanchez Lopera

   
   This file has the implementation of the state machine that integrates all implemented services of the project.
   Moreover, the modules for walking and speech recognition are defined here for more facility.   
   Finally, some other topics and services are called for adaptate the Nao to a specfic situation. 
   For example, the movement of the head, the sensing of tactile buttons and speaking.
   
   The most important functions implemented for the walking module were walkToAruco() and calcNextstep(). 
   This functions and the service /pos_aruco_floor allows Nao to walk towards an object, towards an ArUco marker or 
   simply to follow the centroid of the line received from the vision module.
   
   
   */

/* Libraries */
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <math.h>       /* tan */
#include <string.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"

#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_msgs/ColorRGBA.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tutorial_6/NaoFace.h"

/*OpenCV related includes.*/
#include <cv_bridge/cv_bridge.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"



/* Our messages*/
#include "planner/ChangePosture.h"
#include "planner/DistanceCamFoot.h"
#include "planner/PosArucoFloor.h"
#include "planner/MoveTo.h"
#include "mazeLine/next_step.h"
#include "nao_control_tutorial_3/TrackBall.h"
#include "nao_control_tutorial_3/GrabBall.h"
#include "nao_control_tutorial_3/HandOverBall.h"


/* Macro definitions- */
#define INITIAL_STATE 0
#define WALK 1
#define TRACK_BALL 2
#define GRAB_BALL 3
#define SEARCH_ARUCO_START 4
#define ARUCO_NOT_FOUND 5
#define SPEECH_RECOGNITION 6
#define ARUCO_END 7
#define ASK_NAME 8
#define FACE_RECOG 9
#define HAND_OVER_BALL 10
#define FIND_LINE_AGAIN 11
#define END 12
#define NO_ACTION 13

#define M_PI 3.14159265358979323846  /* pi */
#define ITER_FACTOR 5 //15

using namespace std;
using namespace cv;
using namespace ros;

bool stop_thread=false;

void spinThread()
{
	while(!stop_thread)
	{
		ros::spinOnce();
		//ROS_INFO_STREAM("Spinning the thing!!");
	}
}


/* Global Variable definition*/
std::vector<float> xDeviation;
std::vector<float> yDeviation;
std::vector<float> Dist;
std::vector<float> Angles;
float zObjectInit=0, xObjectInit=0;
float sumDists;
float meanAngles;
float firstAngle;
int counterVoc=0;
float diff;



class Nao_control
{
public:

    /*State machine variables*/
    int currentState;
    int nextState;
    bool standUp;
    bool stopWalking;


    int counter; /*Id for speech functions*/
    vector <string> recText;
    int doneSpeechRecog=0;
    string colorDetect, nameDetect;
    int modeSpeech;
    
    float delta_pitch, delta_yaw;
    float yObjectInit=0;
    

    /* Ros services */
    ros::ServiceClient clientP; // change posture
    ros::ServiceClient clientDCF;// Measure distance camera foot
    ros::ServiceClient clientAruco; // calculate the x,y and theta needed in order to reach the aruco
    ros::ServiceClient clientTB; // Track ball service
    ros::ServiceClient clientGB; // Grab ball service
    ros::ServiceClient clientFR; // Face Recog service
    ros::ServiceClient clientHOB; // Handover ball

    ros::Subscriber tf;
    ros::ServiceClient clientstep;
    ros::Subscriber sensor_data_sub;

	// ros handler
	ros::NodeHandle nh_;

	// subscriber to bumpers states
	ros::Subscriber bumper_sub;

	// subscriber to head tactile states
	ros::Subscriber tactile_sub;

	//publisher for nao speech
	ros::Publisher speech_pub;

	//publisher for nao leds
	ros::Publisher leds_pub;

	//publisher for nao vocabulary parameters
	ros::Publisher voc_params_pub;

	//client for starting speech recognition
	ros::ServiceClient recog_start_srv;

	//client for stoping speech recognition
	ros::ServiceClient recog_stop_srv;

	// subscriber to speech recognition
	ros::Subscriber recog_sub;

	// publisher to nao walking
	ros::Publisher walk_pub;

	//subscriber for foot contact
	ros::Subscriber footContact_sub;

	boost::thread *spin_thread;



	Nao_control()
	{
        /*Start in the initial state*/
        currentState=0;
        nextState=0;
        standUp=false;
        stopWalking=true;

        counter=0;


        /*Service to be called in the planner*/
        clientP = nh_.serviceClient<planner::ChangePosture>("change_posture_service");
        clientDCF = nh_.serviceClient<planner::DistanceCamFoot>("distance_cam_foot_service");
        clientAruco =nh_.serviceClient<planner::PosArucoFloor>("pos_aruco_floor_service");
     
        clientTB = nh_.serviceClient<nao_control_tutorial_3::TrackBall>("track_ball");
        clientGB = nh_.serviceClient<nao_control_tutorial_3::GrabBall>("grab_ball");
        clientHOB= nh_.serviceClient<nao_control_tutorial_3::HandOverBall>("handover_ball");
        
        clientFR = nh_.serviceClient<tutorial_6::NaoFace>("/recognize_face");
        clientstep = nh_.serviceClient<mazeLine::next_step>("/point_next_step");
        

        sensor_data_sub = nh_.subscribe("/joint_states",1, &Nao_control::sensorCallback, this);

		// subscribe to topic bumper and specify that all data will be processed by function bumperCallback
		bumper_sub=nh_.subscribe("/bumper",1, &Nao_control::bumperCallback, this);

		// subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
		tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);

		speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

		leds_pub= nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);

		voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

		recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

		recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

		recog_sub=nh_.subscribe("/word_recognized",1, &Nao_control::speechRecognitionCB, this);

		footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Nao_control::footContactCB, this);

		walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);

		stop_thread=false;
		spin_thread=new boost::thread(&spinThread);
	}
	~Nao_control()
	{
		stop_thread=true;
		sleep(1);
		spin_thread->join();
	}

	void footContactCB(const std_msgs::BoolConstPtr& contact)
	{
        //ROS_INFO_STREAM("contacto pie");
        if(contact->data==true)
        {
          //  ROS_INFO_STREAM("True");
            stopWalking=true;
        }
       else
        {
          //  ROS_INFO_STREAM("False");
            stopWalking=false;
        }
	}

	
	/*
	This callback function compares the recognized word with our vocabulary.
	In this project, we can have colors: "blue", "red","yellow" and our names
	
	When a word is recognized, the flag doneSpeechRecog is set to 1 and the state machine
	will update the current stated
	
	*/
	void speechRecognitionCB(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
	{
		static int len=0;
		int i=0;
		vector <string> color;
		vector <string> names;
		vector<string> rWord=msg->words;
        std_srvs::Empty emptsrv;
		
		color.push_back("yellow");
		color.push_back("blue");
		color.push_back("red");
		
		
        names.push_back("Dani");
		names.push_back("German");
		names.push_back("Francisco");
		
		ROS_INFO_STREAM(rWord.at(0));
		recText.push_back(rWord.at(0));
		len++;

		/*Check if a color was recognized*/
		if(modeSpeech==0){
			for(i=0;i<3;i++){
				if(color[i].compare(recText[len-1]) == 0){
					colorDetect=color[i];
					len=0;
					recog_stop_srv.call(emptsrv);
					recText.clear();
                    doneSpeechRecog=1;
                    ROS_INFO_STREAM("Color Detected "<<colorDetect);
					break;
				}
			}
		}
		else{
			for(i=0;i<3;i++){
				if(names[i].compare(recText[len-1]) == 0){
					nameDetect=names[i];
                    ROS_INFO_STREAM("Name Detected "<<nameDetect);
					len=0;
                    doneSpeechRecog=1;
                    recText.clear();
					recog_stop_srv.call(emptsrv);
					break;
				}
			}
			
		}
		ros::Duration(1).sleep();  

	}
	
	
    void stopWalk()
    {
            geometry_msgs::Pose2D goalPose;
            goalPose.x=0;
            goalPose.y=0;
            goalPose.theta=0;
            walk_pub.publish(goalPose);

     }

	 /* This function publishes the text we want Nao to say */
    void talk( vector <string> textNao){
        naoqi_bridge_msgs::SpeechWithFeedbackActionGoal text;

        for(int i=0;i<textNao.size();i++){
                       ROS_INFO_STREAM(textNao[i]);
                       counter++; /*Increase unique id*/
                       text.goal_id.id=counter;
                       text.goal.say=textNao[i];
                       speech_pub.publish(text);
                       ros::Duration(1).sleep(); /*Pause between words*/
         }


    }

	void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState)
	{
	
	}


	/* Tactile button should be pressed in order to start the Nao*/

  void tactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactileState)
	{
      /*Initial state. If the middle button is pressed,stand up and start looking for start of maze*/
      if(tactileState->button==2 && tactileState->state==1 )
      {  /* Middle tactile button is pressed*/
		standUp=true; /*Start moving*/

		currentState= INITIAL_STATE;
      }

	}
	
	/* This function publishes the desired positions to which the Nao has to walk*/

  void walker(double x, double y, double theta)
  {
      /* Publish desired position (x,y,theta) to the topic */
      geometry_msgs::Pose2D goalPose;

      goalPose.x=x;
      goalPose.y=y;
      goalPose.theta=theta;

     // ROS_INFO_STREAM(goalPose);
      walk_pub.publish(goalPose);

  }

	/*In this function an approximation between the pixel coordinates and coordinates of the real word was made
	  Here an approximated pixel size was found using geometrical analysis.
	*/
    void calcNextStep(int xNextPixel,int yNextPixel, float * thetaLine, float * xNext, float * yNext)
    {
        planner::DistanceCamFoot srvDCF;
        int sizex= 640, sizey=480;
        int xCentered, yCentered;
        float pixelSize, theta;
        static float thetaOld=0;
        static int flag=0; //firs time
        

        srvDCF.request.x=(xNextPixel); //sizex-0.5);
        srvDCF.request.y=(yNextPixel); ///sizey-0.5);
        clientDCF.call(srvDCF); // Distance z

        (*thetaLine)=srvDCF.response.yaw;
        theta=srvDCF.response.yaw;


        xCentered= sizex/2-xNextPixel;
        yCentered= sizey- yNextPixel;
        pixelSize=0.1246/240.0; //0.12

        (*xNext)=pixelSize*(xCentered);
        (*yNext)=pixelSize*(yCentered);

        (*thetaLine)=(xNextPixel-320)*pixelSize;
        diff=(xNextPixel-320);

        //ROS_INFO_STREAM("DIFERENCIA PUNTO Y 320 "<<(xNextPixel-320));

    }

	/*Callback to find out the current position of the head*/
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& jointState)
    {
          delta_pitch= (jointState)->position[1]; // Find angle theta between bottom camera and torso
          delta_yaw=(jointState)->position[0]; // marker rotation

     }

	 /* This function calls the service that transforms coordinates in camera frame to torso frame.
		Then it finds the euclidian distance and the angle theta in which the aruco is found.
	 */
    void walkToAruco(float x, float y, float z, int mode)
    {
        planner::PosArucoFloor srv;
        float xAruco=0,yAruco=0, thetaAruco=0, anglePitch=0;

        srv.request.x=x;
        srv.request.y=y;
        srv.request.z=z;

        ROS_INFO_STREAM("Walk to aruco");

        if(clientAruco.call(srv)) // get aruco coordinates in torso frame
        {
             ROS_INFO_STREAM("x: "<<srv.response.xyz[0]);
             ROS_INFO_STREAM("y: "<<srv.response.xyz[1]);
             ROS_INFO_STREAM("z: "<<srv.response.xyz[2]);


             thetaAruco= (atan2(x,z))-delta_yaw;
             anglePitch= (atan2(y,z));


			 if(mode==0){
                xAruco=srv.response.xyz[0]; // Foot size
                yAruco=srv.response.xyz[1];
				walker(xAruco, yAruco, 0.0);
                ros::Duration(3).sleep();
			 }
			 else{
					ROS_INFO_STREAM("_________- WalktoAruco Mode 1________");
              

					walker(0,0, -(thetaAruco));
					ros::Duration(3).sleep();
					ROS_INFO_STREAM("x: "<<srv.response.xyz[0]);
					ROS_INFO_STREAM("y: "<<srv.response.xyz[1]);
					ROS_INFO_STREAM("z: "<<srv.response.xyz[2]);

                    ROS_INFO_STREAM("WalktoAruco Mode 1_turn "<<thetaAruco);

                    xAruco=srv.response.xyz[0]; // Foot size
                    yAruco=srv.response.xyz[1];
                    
                    if(currentState == SEARCH_ARUCO_START){
                        xAruco=sqrt(xAruco*xAruco+ yAruco*yAruco );

                    }
                    else
                    {

                        xAruco=sqrt(xAruco*xAruco+ yAruco*yAruco )- 0.05;
                    }
                    ROS_INFO_STREAM("WalktoAruco Mode 1_adelante "<<xAruco);
                    walker(xAruco,0, 0);
                    ros::Duration(10).sleep();
                    
                    if(currentState == TRACK_BALL){
                            xObjectInit=xAruco;
                            yObjectInit=yAruco;
                            zObjectInit=-thetaAruco; // No z! but the anglexObjectInit I turned before

                     }				
			 }	
             ROS_INFO_STREAM("End walktoAruco");

        }
        else{

            ROS_INFO_STREAM("service error");

        }

    }

    float mean(float * vector, int len)
    {
        int i=0;
        float sum=0;

        for(i=0;i<len;i++){
            ROS_INFO_STREAM(sum);
            sum+=sum+vector[i];
        }

        sum=sum/len;

        return sum;

    }
    
	
	/* For the speech recognition the first step is to set the vocabulary of words to be recognized*/
    void set_vocabulary(){
		
			std::string id;
			std::vector<string> words_;
            std_srvs::Empty emptsrv;
			


			naoqi_bridge_msgs::SetSpeechVocabularyActionGoal vocabulary;
            counterVoc++;
            id= boost::lexical_cast< std::string >(counterVoc);
            vocabulary.goal_id.id= id;

      
            ROS_INFO_STREAM("Set vocabulary colors done");
            words_.push_back("yellow");
            words_.push_back("blue");
            words_.push_back("red");
    
            ROS_INFO_STREAM("Set vocabulary names done");
            words_.push_back("Dani");
            words_.push_back("German");
            words_.push_back("Francisco");


			vocabulary.goal.words=words_;

            ROS_INFO_STREAM(vocabulary);
			voc_params_pub.publish(vocabulary);

            recog_start_srv.call(emptsrv);
           
          //  ROS_INFO_STREAM("Set vocabulary done");
				
	}

	
	/* This function implements the state machine that integrates all services */

    void main_planner()
	{
        vector <string> text;

        planner::ChangePosture srvP;

        planner::MoveTo srvMove;
        
        nao_control_tutorial_3::TrackBall srvTB;
        nao_control_tutorial_3::GrabBall srvGB;
        nao_control_tutorial_3::HandOverBall srvHOB;
        
        mazeLine::next_step srvNextStep;
        tutorial_6::NaoFace srvFR;
        std_srvs::Empty emptsrv;

        int xNextPixel;
        int yNextPixel;
       
        int turn, boundy, boundw, boundh;

        static float xNext;
        static float yNext;
        float thetaLine;
        string arucoString;
        static int counter=0;
        static int isR=0, isL=0;
        // Track Ball
        static vector<float> position;
        static int objectDone=0;

        // Aruco position
        float x,y,z;

        int isLine=0;

        float exc;
        int iter=0;
        float xVector[ITER_FACTOR]={0};
        static float xFinal=0, yFinal=0 ;
        float yVector[ITER_FACTOR]={0};
        static float stepX[50]={0},stepY[50]={0};
        static int numSteps=0;
        float static sumX=0, sumY=0;
        
        static float xArucoEnd=0, yArucoEnd=0, zArucoEnd=0;
        static int notFoundCounter=0, firstTime =0, lastTurned=0;
        int len=0;


        // Reset
        if(stopWalking==false)
        {
            stopWalk();
            stopWalking=true;
            standUp=false;
            currentState= INITIAL_STATE;
            nextState=INITIAL_STATE;
            ROS_INFO_STREAM("Stop Walking");
        }

		
		
		while(nh_.ok())
		{
            switch(currentState){

                case INITIAL_STATE:
                    if(standUp){
                        ROS_INFO_STREAM("Case INITIALSTATE- Button pressed: Let's start");




                        standUp=false;
                        text.push_back("Hello"); //Use this!
                        talk(text);
                        ros::Duration(1).sleep();
                        srvP.request.desiredPosture=1;
                        srvP.request.angle=30;
                        clientP.call(srvP);
						
                        // Nao preguntar q color!
                        ros::Duration(3).sleep();
                        text.clear();
                        text.push_back("Give me a color"); //Use this!
                        talk(text);
                        text.clear();
                        modeSpeech=0;
                        set_vocabulary();
                        nextState=SPEECH_RECOGNITION;// ;/; //;
                    }
                    else{
                        nextState=currentState; /*If the button is not pressed, stay here*/
                    }
                    break;
                    
                case SPEECH_RECOGNITION:

						if(doneSpeechRecog==1 && modeSpeech==0){ //Color was recognized
							nextState=SEARCH_ARUCO_START;
                            ROS_INFO_STREAM("Speech recognition to search aruco");
                            firstTime=0;
                            doneSpeechRecog=0;

                            srvP.request.desiredPosture=1;
                            srvP.request.angle=30;
                            clientP.call(srvP);
                            ros::Duration(2).sleep();

						}
						else if(doneSpeechRecog==1 && modeSpeech==1){ //name was recognized
							nextState=FACE_RECOG;
                            firstTime=0;
                            doneSpeechRecog=0;
                        }



						
						break;
						

                case SEARCH_ARUCO_START:

                    srvNextStep.request.preference=1;
                    if(clientstep.call(srvNextStep))
                    {
                        

                        if(srvNextStep.response.id==1){ // Aruco found
                             ROS_INFO_STREAM("Aruco found");


                             srvP.request.desiredPosture=4;
                             srvP.request.angle=0;
                             clientP.call(srvP);
                             ros::Duration(1).sleep();

                             srvP.request.desiredPosture=1;
                             srvP.request.angle=30;
                             clientP.call(srvP);
                             ros::Duration(1).sleep();

                             counter=0;
                             x=srvNextStep.response.x;
                             y=srvNextStep.response.y;
                             z=srvNextStep.response.z;
                           
                             walkToAruco(x,y,z,1);
                             ros::Duration(3).sleep();

                             srvP.request.desiredPosture=1;
                             srvP.request.angle=30;
                             clientP.call(srvP);

                             ros::Duration(4).sleep();
                             notFoundCounter=0;
                             nextState=WALK;
                             

                        }
                        else // Aruco not found, take a step, rotate head
                        {
                             nextState= ARUCO_NOT_FOUND;

                        }


                    }
                    else
                    {
                        ROS_INFO_STREAM("Error in German Service");

                    }

                    break;

                case ARUCO_NOT_FOUND:
						ROS_INFO_STREAM("Aruco not found");

						// Rotate head, take one step
						notFoundCounter++;

                        srvP.request.desiredPosture=1;
                        srvP.request.angle=30;
                        clientP.call(srvP);
                        ros::Duration(1).sleep();

                        nextState=SEARCH_ARUCO_START;

						ros::Duration(2).sleep();

                        if(notFoundCounter <= 1)
                        {
							nextState=currentState;
							break; //try two times again
						}
                        else if(notFoundCounter==2){
                            walker(0,0,-10*M_PI/180);
                             ros::Duration(5).sleep();
                             break;
						}
                        else if(notFoundCounter==3)
                        {
                            walker(0,0,30*M_PI/180);
                             ros::Duration(5).sleep();
                             break;
						}
						else{

                            walker(0.09,0,0);
                             ros::Duration(5).sleep();
							notFoundCounter=0;
						}	



                        ros::Duration(3).sleep();


                        break;



                case WALK:
                    ROS_INFO_STREAM("Walking");


                    srvP.request.desiredPosture=5;
                    srvP.request.angle=30;
                    clientP.call(srvP);
                    srvTB.request.color=colorDetect;
                   


                    srvNextStep.request.preference=2;
                    ros::Duration(1).sleep();
                    while(iter<ITER_FACTOR){ // Accuracy: take 10 samples per point
                         /*Look for aruco end*/
                        if(clientstep.call(srvNextStep)){
                                if(srvNextStep.response.id==5){
                                    nextState=ARUCO_END;
                                    xArucoEnd=srvNextStep.response.x;
                                    yArucoEnd=srvNextStep.response.y;
                                    zArucoEnd=srvNextStep.response.z;
                                    break;
                                }
                                xNextPixel=srvNextStep.response.x;
                                yNextPixel=srvNextStep.response.y;
                                turn=srvNextStep.response.turn;

                                ROS_INFO_STREAM("Turn vale"<<turn);

                                boundh=srvNextStep.response.boundh;
                                boundw=srvNextStep.response.boundw;

                                if(((boundh*boundw)>1000) ){ // just big boxes
                                    xVector[iter]=xNextPixel;
                                    yVector[iter]=yNextPixel;
                                    sumX=sumX+xNextPixel;
                                    sumY=sumY+yNextPixel;
                                    iter++;
                                }
                        }
                        else
                        {
                            ROS_INFO_STREAM("Service error German");
                        }
                    }
                    iter=0;
                    // Get the mean
                    xFinal=sumX/ITER_FACTOR;
                    yFinal=sumY/ITER_FACTOR;                 
                
                    
                    sumX=0;
                    sumY=0;
                 
                    // Save the measure
                    calcNextStep(xFinal,yFinal,&thetaLine, &yNext, &xNext); // xNext, Ynext-> in foot coordinates
                    stepX[numSteps]=xNext;
                    stepY[numSteps]=yNext; //+0.0271;

                    exc=((float)boundh/(float)boundw);

                    if(exc>=1.2 ){
                        isLine=1;
                    }else
                    {
                        isLine=0;

                    }

                    //Compare to last step
                    if(numSteps>0){
                       ROS_INFO_STREAM("******************* New Step ***************");
                       ROS_INFO_STREAM("Step X: "<<stepX[numSteps]);
                        ROS_INFO_STREAM("Step Y: "<<stepY[numSteps]);
                           ROS_INFO_STREAM("Step Y-1 : "<<stepY[numSteps-1]);
                        ROS_INFO_STREAM("EXCC: "<<float(exc));
                         ROS_INFO_STREAM("****************************+**************");
                       ros::Duration(3).sleep();



                       if(objectDone==0)
                       {
                           if(clientTB.call(srvTB))
                           {
                               position = srvTB.response.pos;

                               if((position[0]!=-1) && (position[1]!=-1))
                               {

                                       nextState=TRACK_BALL;
                                       break;
                               }

                           }
                        }


                        if(isLine!=1)
                        {

                            if(turn==2)//(stepY[numSteps-1]<stepY[numSteps] )) { //
                             { //  walker(0,0,-90);
                                ROS_INFO_STREAM("90: ");
                                ros::Duration(4).sleep();
                  
                                /*Look for aruco end*/
                                clientstep.call(srvNextStep);
                                if(srvNextStep.response.id==5){
                                    nextState=ARUCO_END;
                                    xArucoEnd=srvNextStep.response.x;
                                    yArucoEnd=srvNextStep.response.y;
                                    zArucoEnd=srvNextStep.response.z;
                                    break;
                                }
                                 ros::Duration(1).sleep();

                                xNextPixel=srvNextStep.response.x;
                                yNextPixel=srvNextStep.response.y;
                                calcNextStep(xNextPixel,yNextPixel, &thetaLine,  &yNext, &xNext); // xNext, Ynext-> in foot coordinates

                                walker(xNext+0.15,0,0);
                                ros::Duration(14).sleep();
                                walker(0,0,90*M_PI/180);
                                ros::Duration(10).sleep();

                                clientstep.call(srvNextStep);
                                if(srvNextStep.response.id==5){
                                    nextState=ARUCO_END;
                                    xArucoEnd=srvNextStep.response.x;
                                    yArucoEnd=srvNextStep.response.y;
                                    zArucoEnd=srvNextStep.response.z;
                                    break;
                                }
                                xNextPixel=srvNextStep.response.x;
                                yNextPixel=srvNextStep.response.y;
                                calcNextStep(xNextPixel,yNextPixel, &thetaLine,  &yNext, &xNext);

                                ROS_INFO_STREAM("Centrarrrr en y "<<thetaLine);
                                walker(0,-thetaLine/2,0);
                                ros::Duration(4).sleep();
   

                                lastTurned=1;


                            }
                            else if( turn==1){ //&& lastTurned==0

                                ROS_INFO_STREAM("-90: ");
                              
                                    /*Look for aruco end*/
                                    if(srvNextStep.response.id==5){
                                        nextState=ARUCO_END;
                                        xArucoEnd=srvNextStep.response.x;
                                        yArucoEnd=srvNextStep.response.y;
                                        zArucoEnd=srvNextStep.response.z;
                                        break;
                                    }
                                     ros::Duration(1).sleep();


                                    clientstep.call(srvNextStep);
                                    if(srvNextStep.response.id==5){
                                        nextState=ARUCO_END;
                                        xArucoEnd=srvNextStep.response.x;
                                        yArucoEnd=srvNextStep.response.y;
                                        zArucoEnd=srvNextStep.response.z;
                                        break;
                                    }
                                    xNextPixel=srvNextStep.response.x;
                                    yNextPixel=srvNextStep.response.y;
                                    calcNextStep(xNextPixel,yNextPixel, &thetaLine, &yNext, &xNext); // xNext, Ynext-> in foot coordinates

                                    walker(xNext+0.15,0,0);
                                    ros::Duration(8).sleep();
                                    walker(0,0,-90*M_PI/180);
                                    ros::Duration(10).sleep();

                                    clientstep.call(srvNextStep);
                                    if(srvNextStep.response.id==5){
                                        nextState=ARUCO_END;
                                        xArucoEnd=srvNextStep.response.x;
                                        yArucoEnd=srvNextStep.response.y;
                                        zArucoEnd=srvNextStep.response.z;
                                        break;
                                    }
                                    xNextPixel=srvNextStep.response.x;
                                    yNextPixel=srvNextStep.response.y;
                                    calcNextStep(xNextPixel,yNextPixel, &thetaLine,  &yNext, &xNext);

                                    ROS_INFO_STREAM("Centrarrrr en y "<<thetaLine);
                                    walker(0,-thetaLine/2,0);
                                    ros::Duration(4).sleep();

                                    lastTurned=1;

                            }
                            else{

                                walker(0,-0.05,0);
                            }
                        }
                        
                        else{
                           //  walker(stepX[numSteps],0,0);
                             ROS_INFO_STREAM("Straigth: ");



                             /*Look for aruco end*/
                             if(srvNextStep.response.id==5){
                                 nextState=ARUCO_END;
                                 xArucoEnd=srvNextStep.response.x;
                                 yArucoEnd=srvNextStep.response.y;
                                 zArucoEnd=srvNextStep.response.z;
                                 break;
                             }
                             ROS_INFO_STREAM("_____________ yNext: "<< yNext);
                             ROS_INFO_STREAM("_____________ xNext: "<<stepX[numSteps]);
                               ROS_INFO_STREAM("_____________ Theta: "<< thetaLine);

                             ROS_INFO_STREAM("Centrarrrr en y "<<thetaLine);
                             walker(0,-thetaLine,0);
                             ros::Duration(4).sleep();
                             if(diff>20)
                             {
                                 walker(0,0,-10*M_PI/180);
                             }
                             if(diff<-20)
                             {
                                 walker(0,0,10*M_PI/180);
                             }
                             ros::Duration(2).sleep();

                             //2da correccion
                             clientstep.call(srvNextStep);
                             if(srvNextStep.response.id==5){
                                 nextState=ARUCO_END;
                                 xArucoEnd=srvNextStep.response.x;
                                 yArucoEnd=srvNextStep.response.y;
                                 zArucoEnd=srvNextStep.response.z;
                                 break;
                             }
                             xNextPixel=srvNextStep.response.x;
                             yNextPixel=srvNextStep.response.y;
                             calcNextStep(xNextPixel,yNextPixel, &thetaLine,  &yNext, &xNext);

                             walker(0,-(thetaLine/2),0);
                             ros::Duration(2).sleep();
 

                             ros::Duration(2).sleep();*/
                             walker(stepX[numSteps],0,0);
                             ros::Duration(4).sleep();

                             /*Look for ball*/
                             if(objectDone==0){
                                 ros::Duration(2).sleep();
                                 lastTurned=0;
                                 if(clientTB.call(srvTB))
                                 {
                                     position = srvTB.response.pos;

                                     if((position[0]!=-1) && (position[1]!=-1))
                                     {

                                                nextState=TRACK_BALL;
                                            break;
                                     }

                                 }
                             }
                             isLine=0;
						}
                    }
                    else{ // First step
                        ROS_INFO_STREAM("First step: ");
                        ROS_INFO_STREAM(stepX[numSteps]);
                        ROS_INFO_STREAM(stepY[numSteps]);

                        isLine=0;

                        ROS_INFO_STREAM("Centrarrrr en y "<<thetaLine);
                            walker(0,-thetaLine,0);
                       // }
                        ros::Duration(4).sleep();
                        if(diff>20)
                        {
                            walker(0,0,-10*M_PI/180);
                        }
                        if(diff<-20)
                        {
                            walker(0,0,10*M_PI/180);
                        }
                        ros::Duration(5).sleep();
                        walker(stepX[numSteps],0,0);

                    }


                    numSteps++;

                    nextState= WALK;
                    break;

            case TRACK_BALL:
					ROS_INFO_STREAM("_______ Track Ball ________");

					if(position[2]==-1 && position[0]!=-1 && position[1]!=-1  )
					{
					   ROS_INFO_STREAM(" Caminando a ball ");

                          position[0]=position[0];
                          position[1]=position[1];
						  walker(position[0],position[1],0);
						  ros::Duration(5).sleep();
						 
						  if(clientTB.call(srvTB))
						  {
							  position = srvTB.response.pos;
						  }

					  Dist.push_back( sqrt( position[0]*position[0] + position[1]*position[1] ) );

                      Angles.push_back(atan(position[1]/position[0]));
                      for (std::vector<float>::const_iterator i = Dist.begin(); i != Dist.end(); ++i)
                      {
                          float aux=*i;
                          ROS_INFO_STREAM("Dist Ball: "<<aux);
                      }

                      for (std::vector<float>::const_iterator i = Angles.begin(); i != Angles.end(); ++i)
                      {
                          float aux=*i;
                          ROS_INFO_STREAM("Angles Ball: "<<aux);
                      }

					  nextState=TRACK_BALL;
					}
					else{ //Aruco!
					  ROS_INFO_STREAM(" Caminando a ARUCO ball ");
                      walkToAruco(position[0],position[1],position[2],1);
                      Angles.push_back(zObjectInit);
					  ros::Duration(5).sleep();
                      Dist.push_back(xObjectInit);

                      for (std::vector<float>::const_iterator i = Dist.begin(); i != Dist.end(); ++i)
                      {
                          float aux=*i;
                          ROS_INFO_STREAM("Dist Aruco: "<<aux);
                      }

                      for (std::vector<float>::const_iterator i = Angles.begin(); i != Angles.end(); ++i)
                      {
                          float aux=*i;
                          ROS_INFO_STREAM("Angles Aruco: "<<aux);
                      }


					  nextState=GRAB_BALL;
					}

					ROS_INFO_STREAM("------------------------End Track Ball");
					break;

			case GRAB_BALL:
					ROS_INFO_STREAM("Grab Ball");
					ros::Duration(5).sleep();


					srvGB.request.pos_ini.clear();

					srvGB.request.pos_ini.push_back(0.10);//x
					srvGB.request.pos_ini.push_back(0);//y
                    srvGB.request.pos_ini.push_back(-0.03);//z

					if(clientGB.call(srvGB))
					{
					  ROS_INFO_STREAM("All good :), second service being called");
					  ros::Duration(10).sleep();
					  nextState=FIND_LINE_AGAIN;
					  objectDone=1;
					}
					else
					{
					  ROS_INFO_STREAM("Problem with second service GrabBall");
					}

					break;
            
                case FIND_LINE_AGAIN:
					ROS_INFO_STREAM("Find Line again");
                    sumDists=0.0;

					for(std::vector<int>::size_type i = 0; i != Dist.size(); i++)
					{
					   std::cout << Dist[i];
                       ROS_INFO_STREAM("Dist i will come back"<<Dist[i]);
                       sumDists=sumDists+ Dist[i];
					}

                    meanAngles=Angles[0];
                    for(std::vector<int>::size_type i = 0; i != Angles.size(); i++)
                    {
                      if(Angles[i]> meanAngles)
                          meanAngles=Angles[i];
                    }
                    ROS_INFO_STREAM("Angulo a girar"<<meanAngles);
                    ROS_INFO_STREAM("Angulo q giro al aruco"<<zObjectInit);

                    walker(-0.20,0,0);
                    Duration(9).sleep();
					walker(0,0, 180*M_PI/180);
					Duration(14).sleep();
                      ROS_INFO_STREAM("Dist sum will come back"<<sumDists);
                    walker(sumDists-0.20,0,0);
					Duration(10).sleep();
                    walker(0,0, 180*M_PI/180);
					Duration(14).sleep();
                    walker(0,0, -meanAngles);
                    Duration(14).sleep();
                    objectDone=1;

                  
                    ros::Duration(2).sleep();

					ROS_INFO_STREAM("FINAL de find line");


					nextState=WALK;
					break;
						
                  case ARUCO_END: // Aruco at the end of the maze was found
						ROS_INFO_STREAM("Aruco End");


						// Locate aruco again.
						 walkToAruco(xArucoEnd,yArucoEnd,zArucoEnd,0);
						 ros::Duration(2).sleep();
						 //if(currentState!=INITIAL_STATE){
						 nextState=ASK_NAME;
						 break;
                        
                  case ASK_NAME:
						// Nao preguntar q color!
                        ros::Duration(1).sleep();
                        text.clear();
                        if(objectDone==0)
                        {
                            text.push_back("sorry.   I do not have any ball. Who wanted it?"); //Use this!
                            ros::Duration(5).sleep();
                        }
                        else
                            text.push_back("Who wants this ball?"); //Use this!
                        talk(text);
                        text.clear();
                        modeSpeech=1;
                        set_vocabulary();                       
                        ros::Duration(2).sleep();
                         
                        nextState=SPEECH_RECOGNITION;
						
						break;

                  case FACE_RECOG:


                        
                        if(notFoundCounter==0){
							ros::Duration(4).sleep();
							srvP.request.desiredPosture=3;
                            srvP.request.angle=-30;
							clientP.call(srvP);
						}



                        // Face recognition
                        srvFR.request.name=nameDetect;
                        ROS_INFO_STREAM(" Face Recognition "<<nameDetect);

                        for(int i=0; i<8;i++){
                            if(clientFR.call(srvFR)){
                                if(srvFR.response.inImage== true){

                                        // Walk to Aruco
                                        x=srvNextStep.response.x;
                                        y=srvNextStep.response.y;
                                        z=srvNextStep.response.z;


                                        ROS_INFO_STREAM("I find "<<srvFR.response.personInImage);

                                        walkToAruco(x,y,z,1);
                                        ros::Duration(5).sleep();

                                        nextState= HAND_OVER_BALL;
                                        break;
                                    // End to aruco
                                }
                            }

                            else{
                                ROS_INFO_STREAM("Error face recognition service");
                            }
                        }

                        if(nextState==HAND_OVER_BALL)
                            break;
                        else
                        {

                            ROS_INFO_STREAM("No encuentro Persona");
                            notFoundCounter++;

                            if(notFoundCounter <= 2){
                                nextState=currentState;
                                break; //try two times again
                            }
                            else if(notFoundCounter==3){
                                walker(0,0.0, 70*M_PI/180);
                                ros::Duration(10).sleep();
                            }
                            else if(notFoundCounter==4){
                                walker(0,0.0, -140*M_PI/180);
                                ros::Duration(10).sleep();
                            }
                            else{
                                walker(0,0.03,0);
                                ros::Duration(10).sleep();
                                notFoundCounter=0;
                            }
                            nextState=FACE_RECOG;
                            break;

                        }


						

					case HAND_OVER_BALL:
							ROS_INFO_STREAM("Hand over ball");
                            ros::Duration(4).sleep();
							// Call service
							if(clientHOB.call(srvHOB))
							{
								ROS_INFO_STREAM("All good :), second service being called");
                                ros::Duration(4).sleep();
					
							}
							else
							{
								ROS_INFO_STREAM("Problem with third service HandOverBall");
							}
							if(currentState!=INITIAL_STATE)
							{
								nextState=END;
							}							
							
							break;
					case END:
                 
							// Change Posture
							srvP.request.desiredPosture=4; //Init
							srvP.request.angle=30;
							clientP.call(srvP);
							ros::Duration(5).sleep();
							// Nao says "Im done"
							text.push_back("I am done"); //Use this!
							talk(text);
                            text.clear();
							nextState=NO_ACTION;
					
							break;
                    default:

                            break;

            }
            currentState=nextState;
		}
	}



};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "planner");

	ros::NodeHandle n;
	ros::Rate rate_sleep(20);
	Nao_control ic;

    ic.main_planner();
	return 0;

}

