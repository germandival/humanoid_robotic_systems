/**************************************************************************/
/*                                                                        */
/* Date: 20.12.2017                                                       */
/* Author: German Diez Valencia    germandiezvalencia@gmail.com           */
/* IndiaNAO johnes                                                        */
/* Group A                                                                */
/* Walkin throug the maze                                                 */
/* Humanoid Robotic Systems                                               */
/**************************************************************************/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
//#include <naoqi_bridge_msgs/TactileTouch.h>
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

#include <cv_bridge/cv_bridge.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

/*Aruco markers*/
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include"mazeLine/next_step.h"

using namespace aruco;

using namespace std;
using namespace cv;
bool stop_thread=false;

const int alpha_slider_max = 255;
int alpha_slider_threshold;
int alpha_slider_offset;
double alpha;
double beta;

Mat dist(1,5,CV_32FC1);
Mat cameraP(3,3,CV_32FC1);



const int slider_max = 255;
const int slider_max_channel = 2;
int minH=0;
int maxH=0;
int minS=0;
int maxS=0;
int minV=0;
int maxV=0;
int valselect=0;
int valselectED=0;
int erodeval=0;
int dilateval=0;
int sizeDilate=0;
int sizeErode=0;
int erosionType;
int dilationType;
int thresh = 100;

char tminH[10];
char tmaxH[10];
char tminS[10];
char tmaxS[10];
char tminV[10];
char tmaxV[10];
char selectHSV[10];
char selectED[10];

char tdilate[20];
char terode[20];
char tSizeDilate[20];
char tSizeErode[20];


void spinThread()
{
	while(!stop_thread)
	{
		ros::spinOnce();
		//ROS_INFO_STREAM("Spinning the thing!!");
	}
}

void on_trackbar( int, void* )
{
 /* No need of performing actions into the callback function*/

}
class Nao_control
{
public:
    string oldBumper;
    int counter;
    vector <string> recText;
    bool stopWalking; /*This flag is activated when the contact with the foot is lost*/

    float sequencex [8];
    float sequencey [8];
    float sequencet [8];
    bool walkStart;

    float Tvecx;
    float Tvecy;
    float Tvecz;
    float Rvecxthetha;
    float Rvecphi;
    float Rvecsci;

    float TvecxTop;
    float TvecyTop;
    float TveczTop;
    float RvecxthethaTop;
    float RvecphiTop;
    float RvecsciTop;

    int arucoStatus,arucoStatusTop;
    int arucoID;

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
    ros::Publisher leds_cancel;

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

    //subscriber for the bottom camera
    ros::Subscriber img_rosB,img_rosT;
    Mat image;

    ros::ServiceServer service = nh_.advertiseService("/point_next_step", &Nao_control::next_step, this);

	boost::thread *spin_thread;
    int ledid;
    int stepx;
    int stepy;
    int turn,boundh,boundw;
	Nao_control()
	{
        oldBumper="first";
        counter=0;
        stopWalking=true;
        walkStart=false;
        /*Moving the robot in a line 500m and turning to left and right in order to describe a square*/
        float auxsequencex [8]= {0.5,0,    0.5, 0,   0.5 , 0,  0.5, 0};
        float auxsequencey [8]= {0 , 0,    0,   0,   0 , 0,    0,   0};
        float auxsequencet [8]= {0 ,-1.57, 0,   -1.57,0 ,-1.57, 0,   -1.57};

        /* Array with sequence of movements*/
        for(int j=0; j<8;j++){
            sequencex[j]= auxsequencex[j];
            sequencey[j]= auxsequencey[j];
            sequencet[j]= auxsequencet[j];
        }
        dist.at<float>(0,0)=-0.0648763971625288;
        dist.at<float>(1,0)=0.0612520196884308;
        dist.at<float>(2,0)=0.0038281538281731;
        dist.at<float>(3,0)=-0.00551104078371959;

        //camera matrix
        cameraP.at<float>(0,0)=558.570339530768;
        cameraP.at<float>(0,1)=0.000000;
        cameraP.at<float>(0,2)=308.885375457296;
        cameraP.at<float>(1,0)=0.000000;
        cameraP.at<float>(1,1)=556.122943034837;
        cameraP.at<float>(1,2)=247.600724811385;
        cameraP.at<float>(2,0)=0.000000;
        cameraP.at<float>(2,1)=0.000000;
        cameraP.at<float>(2,2)=1.000000;

        // subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
        tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);
        // Advertise to topic speech_action in order to publish later
        speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);
         // Advertise to topic blink/goal in order to publish later
        leds_pub= nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);
        leds_cancel= nh_.advertise<actionlib_msgs::GoalID>("/blink/cancel", 1);

        //subscribe to the bottom camera
        img_rosB= nh_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1, &Nao_control::imgCallbackBottom, this);
        img_rosT= nh_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &Nao_control::imgCallbackTop, this);

        voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);
        // Service to start speech recognition
        recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");
        // Service to stop speech recognition
        recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

        footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Nao_control::footContactCB, this);
        // Advertise to topic cmd_pose to control the walking of the robot
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

    bool next_step(mazeLine::next_step::Request & req, mazeLine::next_step::Response &res)
    {
    	// the request is looking for the initial aruco
        if(req.preference==1)
        {
            if (arucoStatus==1 && arucoID==1)
            {
                res.x=Tvecx;
                res.y=Tvecy;
                res.z=Tvecz;
                res.tetha=Rvecxthetha;
                res.sci=Rvecsci;
                res.phi=Rvecphi;
                res.id=1;

                ROS_INFO_STREAM(res);
                return true;
            }
            else
            {
                res.id=0;
                return true;
            }
        }
        else if(req.preference!=10)// the preference is different to 10 means we are not lookinf for the aruco ball
        {
            if (arucoStatus==1 && arucoID==2)// We are not explicitelly looking for aruco ball (aruco 10) and we find just one aruco with the end of the maze
            {
                res.x=Tvecx;
                res.y=Tvecy;
                res.z=Tvecz;
                res.tetha=Rvecxthetha;
                res.sci=Rvecsci;
                res.phi=Rvecphi;
                res.id=5;

                ROS_INFO_STREAM(res);
                return true;
            }
            //we dont find the endo of the maze so we return the direction of the next step and the sugestion to turn left, right or continue stright
            findLine(image,req.preference);
            res.x=stepx;
            res.y=stepy;
            res.boundh=boundh;
            res.boundw=boundw;
            res.turn=turn;
            res.id=req.preference;
            ROS_INFO_STREAM(res);
            return true;
        }
        else
        {
            if (arucoStatusTop==1 && arucoID==10)// if we are looking for a aruco ball and we find the aruco with ID 10
            {
                res.x=TvecxTop;
                res.y=TvecyTop;
                res.z=TveczTop;
                res.tetha=RvecxthethaTop;
                res.sci=RvecsciTop;
                res.phi=RvecphiTop;
                res.id=10;

                ROS_INFO_STREAM(res);
                return true;
            }
            else
            {
                res.id=0;
                return true;
            }
        }
    }

    void footContactCB(const std_msgs::BoolConstPtr& contact)
    {
        ROS_INFO_STREAM("Foot contact");
        if(contact->data==true) /*If the foot is not in contact with the ground, stop walking*/
        {
            ROS_INFO_STREAM("True");
            stopWalking=true;
        }
       else
        {
            ROS_INFO_STREAM("False"); /* Otherwise the robot can walk */
            stopWalking=false;
        }
    }


    /* Callback for the bottom camera */

    void imgCallbackTop(const sensor_msgs::Image& img_raw)//callback for the top image. We dont try to find lines on the top image for that reason this callback just try to find arucos.
    {
	
        Mat topImage;
        arucoStatusTop=0;

        /* Create the bridge between ros and opencv format*/
        cv_bridge::CvImagePtr  cv_img_ptr;
        /* Create matrix for each result image*/

        Mat src_gray;
        vector<Vec3f> circles;

        /*Convert from ROS image to OpenCv image format*/
        try{
           cv_img_ptr = cv_bridge::toCvCopy(img_raw,"bgr8");
           cv_img_ptr->image.copyTo(topImage);

        }
        catch (cv_bridge::Exception& e){
           ROS_ERROR("cv_bridge exception from camera: %s", e.what());
           return;
        }

        MarkerDetector MDetector;
        vector<Marker> Markers; /*List with corners of the detected marker*/
        Mat aux;
        cvtColor(topImage, aux, CV_BGR2GRAY);
        MDetector.detect(aux,Markers); /*Image is where the markers are going to be detected */

        int  sizem= Markers.size();
        
        if(sizem==1)
        {
            arucoStatusTop=1;
            arucoID=Markers[0].id;
            Markers[0].calculateExtrinsics(0.10,cameraP,dist); /* Calculate the rotation (rvec) and translation vectors (tvec)*/
            Markers[0].draw(topImage,Scalar(0,0,255),2);
            TvecxTop=Markers[0].Tvec.at<Vec3f>(0,0)[0];
            TvecyTop=Markers[0].Tvec.at<Vec3f>(0,0)[1];
            TveczTop=Markers[0].Tvec.at<Vec3f>(0,0)[2];
            RvecxthethaTop=Markers[0].Rvec.at<Vec3f>(0,0)[3];
            RvecsciTop=Markers[0].Rvec.at<Vec3f>(0,0)[4];
            RvecphiTop=Markers[0].Rvec.at<Vec3f>(0,0)[5];
        }



        imshow("Top camera", topImage);
        waitKey(3);

    }
    void imgCallbackBottom(const sensor_msgs::Image& img_raw)// This function finds arucos in the bottom camera and lines in the maze
    {
        arucoStatus=0;
        
        /* Create the bridge between ros and opencv format*/
        cv_bridge::CvImagePtr  cv_img_ptr;
        /* Create matrix for each result image*/

        Mat src_gray;
        vector<Vec3f> circles;

        /*Convert from ROS image to OpenCv image format*/
        try{
           cv_img_ptr = cv_bridge::toCvCopy(img_raw,"bgr8");
           cv_img_ptr->image.copyTo(image);

        }
        catch (cv_bridge::Exception& e){
           ROS_ERROR("cv_bridge exception from camera: %s", e.what());
           return;
        }

        MarkerDetector MDetector;
        vector<Marker> Markers; /*List with corners of the detected marker*/

        Mat aux;
        cvtColor(image, aux, CV_BGR2GRAY);
        MDetector.detect(aux,Markers); /*Image is where the markers are going to be detected */

        //MDetector.detect(image,Markers); /*Image is where the markers are going to be detected */

        int  sizem= Markers.size();
        //ROS_WARN_STREAM("sizem: " << sizem);
        if(sizem==1)
        {
            arucoStatus=1;
            arucoID=Markers[0].id;
            Markers[0].calculateExtrinsics(0.10,cameraP,dist); /* Calculate the rotation (rvec) and translation vectors (tvec)*/
            Markers[0].draw(image,Scalar(0,0,255),2);
            Tvecx=Markers[0].Tvec.at<Vec3f>(0,0)[0];
            Tvecy=Markers[0].Tvec.at<Vec3f>(0,0)[1];
            Tvecz=Markers[0].Tvec.at<Vec3f>(0,0)[2];
            Rvecxthetha=Markers[0].Rvec.at<Vec3f>(0,0)[3];
            Rvecsci=Markers[0].Rvec.at<Vec3f>(0,0)[4];
            Rvecphi=Markers[0].Rvec.at<Vec3f>(0,0)[5];

        }
        //calls the find line function with the preference 42 for visualization 
        findLine(image,42);
        imshow("p", image);
        waitKey(3);



    }
    void find_contour( Mat src_gray,int preference) //function to find contours in the maze
    {
        int margin=50;
        Scalar color2( 255,255,0);
        //ROS_INFO_STREAM(src_gray.size().width);
        for(int fl=0;fl<src_gray.size().width-1; fl++)
        {
            for(int cl=0;cl<src_gray.size().height-1;cl++)
            {
                if (cl <=margin || cl>=src_gray.size().height-margin || fl <=margin || fl>=src_gray.size().width-margin)
                {
                    //ROS_INFO_STREAM(fl);
                    src_gray.at<Vec3b>(cl,fl)[0]=0;
                    src_gray.at<Vec3b>(cl,fl)[1]=0;
                    src_gray.at<Vec3b>(cl,fl)[2]=0;
                }
            }
        }

        int largest_area=0;/*auxiliar variable to select the bigest contour (the contour with more area)*/
        int largest_contour_index=0;/*variable to save the position of the bigest contour in the contours array*/
        Rect bounding_rect,rec_left,rec_righ;/*rectangle over the bgest contour*/
        std::vector< vector<Point> > contours; /*Vector for storing contour*/
        std::vector<Vec4i> hierarchy;  /*Vector for storing contour hierarchy, the countour also have internal countours the opencv findContours not only find contour in the image but also stablish a relational hierarcy between them */
        Mat canny_output, thr; /* variables to store canny_output: the border images and thr: the grayscale image*/
        Canny( src_gray, thr, thresh, thresh*2, 3 );/*extract the border image*/

        findContours( thr, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); /*Find the contours in the image*/
        Scalar color( 255,255,0);
        for( int i = 0; i< contours.size(); i++ ) /*iterate through each contour to find the one with bigest area*/
        {
            double a=contourArea( contours[i],false);  /*  Find the area of contour of each contour*/
            if(a>largest_area)/* update the bigest area*/
            {
                drawContours( src_gray, contours,i, color, CV_FILLED, 8, hierarchy );
                largest_area=a;
                largest_contour_index=i;                /*Store the index of largest contour*/
                bounding_rect=boundingRect(contours[i]); /* Find the bounding rectangle for biggest contour*/
            }

        }
        drawContours( src_gray, contours,largest_contour_index, color, CV_FILLED, 8, hierarchy ); /* Draw the largest contour using previously stored index.*/

        rectangle(src_gray, bounding_rect,  Scalar(0,255,0),1, 8,0);  /*draw the bounding rect over the largest contour*/

        try
        {
            vector<Moments> mu(contours.size() ); /*Vector for storing the mass center of the contours*/
            /*Get the moments of the contours*/
            for( int i = 0; i < contours.size(); i++ )
            {
                mu[i] = moments( contours[i], false );
            }

            /*  Get the mass centers: using mu moments*/
            vector<Point2f> mc( contours.size() );
            for( int i = 0; i < contours.size(); i++ )
            {
                mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
            }
            /*Draw a circle in the center of mass of the bigest contour*/
            if(contours.size()>0)
            {
                circle( src_gray, mc[largest_contour_index], 4, color, -1, 8, 0 );
            }
            //ROS_INFO_STREAM("The preference is "<<preference);
            if(preference==42)
            {
                imshow( "Blob Extraction", src_gray );
            }
            else
            {
                if(preference==2) // the preference is to follow a stright line
                {
                    stepx=bounding_rect.x+bounding_rect.width/2;
                    stepy=bounding_rect.y+bounding_rect.height/2;
                }
                else if(preference==3)// the preference is to right
                {
                    stepx=bounding_rect.x+bounding_rect.width*0.7;//to advance slightly less
                    stepy=bounding_rect.y+bounding_rect.height*0.7;//to advance slightly more to the right
                }
                else if(preference==4)// the preference is to left
                {
                    stepx=bounding_rect.x+bounding_rect.width*0.3;//to advance slightly less
                    stepy=bounding_rect.y+bounding_rect.height*0.7;//to advance slightly more to the right
                }
                rec_left.x=bounding_rect.x;
                rec_left.y=bounding_rect.y;
                rec_left.height=bounding_rect.height;
                rec_left.width=bounding_rect.width/2;


                rec_righ.x=bounding_rect.x+bounding_rect.width/2;
                rec_righ.y=bounding_rect.y;
                rec_righ.height=bounding_rect.height;
                rec_righ.width=bounding_rect.width/2;

                Mat crop_left = src_gray(rec_left);
                Mat crop_righ = src_gray(rec_righ);

                rectangle(src_gray, rec_left,  Scalar(255,0,0),1, 8,0);  /*draw the bounding rect left*/
                rectangle(src_gray, rec_righ,  Scalar(255,0,255),1, 8,0);  /*draw the bounding rect right*/

		//for sugesting the left or right turn we divide the image in two vertical parts and check the number of whtie point in each half of the 
                int count_white_left = 0;
                for( int y = 0; y < crop_left.rows; y++ )
                {
                    for( int x = 0; x < crop_left.cols; x++ )
                    {
                          if ( crop_left.at<cv::Vec3b>(y,x) == cv::Vec3b(255,255,255) )
                          {
                            count_white_left++;
                          }
                    }
                }
                int count_white_right = 0;
                for( int y = 0; y < crop_righ.rows; y++ )
                {
                    for( int x = 0; x < crop_righ.cols; x++ )
                    {
                          if ( crop_righ.at<cv::Vec3b>(y,x) == cv::Vec3b(255,255,255) )
                          {
                            count_white_right++;
                          }
                    }
                }

                turn=0;
                //threshold to identify if the right half or the left half has way more elements than the other this is used to suggest the turn of the robot 
                if(count_white_left>count_white_right+5000)
                {
                    ROS_INFO_STREAM("------------------------------------turn right");
                    //putText(src_gray, "turn right.", cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
                    turn=1;
                }
                if( count_white_right>count_white_left+5000)
                {
                    ROS_INFO_STREAM("------------------------------------turn left");
                    //putText(src_gray, "turn left.", cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
                    turn=2;
                }
                boundh=bounding_rect.height;
                boundw=bounding_rect.width;


                imshow( "Blob Extraction", src_gray );
                imshow( "Blob left", crop_left );
                imshow( "Blob right", crop_righ );
            }



        }
        catch (int e)
        {
           ROS_INFO_STREAM("ERROR: ");
        }



    }
    void findLine(cv::Mat image,int preference)//morphological operations to identify a black line into a white background
    {
        /* Create matrix for each result image*/
        Mat image_gray;
        Mat image_Line;
        Mat mazeBack;
        Mat line_in_Maze;
        Mat image_bitwise;
        Mat image_erode;
        Mat image_dilate;

        cvtColor(image, image_gray, CV_BGR2GRAY);
       // imshow("gray", image_gray);

        int sizeDilate=15;
        int sizeErode=3;
        Mat elementDilate= getStructuringElement(MORPH_RECT,Size(2*sizeDilate+1,2*sizeDilate+1),Point(sizeDilate,sizeDilate));
        Mat elementErode= getStructuringElement(MORPH_RECT,Size(2*sizeErode+1,2*sizeErode+1),Point(sizeErode,sizeErode));
        cv::threshold(image_gray, mazeBack, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);// is necesary to use otsu to have a dinamic threshold based on the light conditions of the image
        dilate(mazeBack, mazeBack, elementDilate);
        erode(mazeBack, mazeBack, elementDilate);
        erode(mazeBack, mazeBack, elementDilate);
        imshow("Otsu", mazeBack);

        cv::threshold(image_gray, image_Line, 60, 255, CV_THRESH_BINARY_INV);
        imshow("line", image_Line);

        bitwise_and(mazeBack, image_Line, line_in_Maze);
        sizeDilate=15;
        elementDilate= getStructuringElement(MORPH_RECT,Size(2*sizeDilate+1,2*sizeDilate+1),Point(sizeDilate,sizeDilate));
        dilate(line_in_Maze, line_in_Maze, elementDilate);
        dilate(line_in_Maze, line_in_Maze, elementDilate);
        erode(line_in_Maze, line_in_Maze, elementErode);
        erode(line_in_Maze, line_in_Maze, elementErode);
        imshow("line_in_Maze", line_in_Maze);

        /*Apply the binary mask into the colored image*/
        cvtColor(line_in_Maze, line_in_Maze, CV_GRAY2BGR);
        bitwise_and(image,line_in_Maze,image_bitwise);
        //imshow("Image_bitwise",image_bitwise);
        find_contour( line_in_Maze,preference);
        waitKey(1);

    }
  void tactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactileState)
    {


        if(tactileState->button==1 && tactileState->state==1){ /* Front tactile button is pressed*/

            walkStart=true;


        }

        if(tactileState->button==2 && tactileState->state==1 ){  /* Middle tactile button is pressed*/
            walkStart=false;

        }

        if(tactileState->button==3 && tactileState->state==1 ){  /* Back tactile button is pressed*/
        }

        ROS_INFO_STREAM(walkStart);
    }


    void main_loop()
    {
       // ros::Rate rate_sleep(10000);
        int currMov=0;
        while(nh_.ok())
        {
            if(stopWalking==true) /*If the foot no longer contact the ground*/
            {
                stopWalk();

            }
            else
            {
                walker(  sequencex[currMov], sequencey[currMov],  sequencet[currMov]); /*Moving the robot following the desired sequence: square */
                currMov++;
                if(currMov>=8) /*Restart the sequence*/
                {
                    currMov=0;
                }
            }

            ros::Duration(3).sleep();
        }
    }

    void walker(double x, double y, double theta)
    {
        /* Publish desired position (x,y,theta) to the topic */
        geometry_msgs::Pose2D goalPose;

        goalPose.x=x;
        goalPose.y=y;
        goalPose.theta=theta;

        ROS_INFO_STREAM(goalPose);
        walk_pub.publish(goalPose);

    }

    void stopWalk()
    {

        geometry_msgs::Pose2D goalPose;

        goalPose.x=0;
        goalPose.y=0;
        goalPose.theta=0;

        walk_pub.publish(goalPose);


    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Maze_line");



    ROS_INFO_STREAM("HELLO !");
    Nao_control ic;
    ros::Rate rate_sleep(20);
    //ic.main_loop();
    ros::spin();
	return 0;

}

