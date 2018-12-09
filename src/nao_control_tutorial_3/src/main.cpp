/*
 * Group C - Humanoid Robotic Systems - Tutorial 7
 * Date: 01.12.2017
 * Authors: Sa Li, Francisco Zurita
 */

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
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/affine.hpp>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/dictionary.h>
#include <aruco/posetracker.h>
#include <aruco/markerdetector.h>
#include "nao_control_tutorial_3/TrackBall.h"
#include "nao_control_tutorial_3/GrabBall.h"
#include "nao_control_tutorial_3/HandOverBall.h"


using namespace std;
using namespace ros;
using namespace cv;
using namespace aruco;

// Global variables
bool stop_thread=false;
static const std::string OPENCV_WINDOW = "Original image";
static const std::string OPENCV_WINDOW_TOP_CAMERA = "Top camera";
Mat dist(1,5,CV_32FC1);
Mat cameraP(3,3,CV_32FC1);

/*---------from trackbars-------*/
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
int thresh = 100;

char tminH[10];
char tmaxH[10];
char tminS[10];
char tmaxS[10];
char tminV[10];
char tmaxV[10];
char selectHSV[10];
char selectED[10];



void spinThread()
{
    while(!stop_thread)
    {
        ros::spinOnce();
        //ROS_INFO_STREAM("Spinning the thing!!");
    }
}

void on_trackbar(int, void*)
{

}


class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;

    boost::thread *spin_thread;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;




    Nao_control(): it_(nh_)
    {
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
          &Nao_control::imageCb_top, this);

        // Create windows for images and properly locate them
        cv::namedWindow(OPENCV_WINDOW_TOP_CAMERA);

        dist=Mat(1,5,CV_32FC1);
        cameraP=Mat(3,3,CV_32FC1);

        dist.at<float>(0,0)=-0.066494;
        dist.at<float>(0,1)=0.095481;
        dist.at<float>(0,2)=-0.000279;
        dist.at<float>(0,3)=0.002292;
        dist.at<float>(0,4)=0.000000;


        cameraP.at<float>(0,0)=551.543059;
        cameraP.at<float>(0,1)=0.000000;
        cameraP.at<float>(0,2)=327.382898;
        cameraP.at<float>(1,0)=0.000000;
        cameraP.at<float>(1,1)=553.736023;
        cameraP.at<float>(1,2)=225.026380;
        cameraP.at<float>(2,0)=0.000000;
        cameraP.at<float>(2,1)=0.000000;
        cameraP.at<float>(2,2)=1.000000;
        stop_thread=false;
        spin_thread=new boost::thread(&spinThread);

    }
    ~Nao_control()
    {
        cv::destroyWindow(OPENCV_WINDOW_TOP_CAMERA);
        sleep(1);
        stop_thread=true;
        sleep(1);
        spin_thread->join();

    }

    void imageCb_top(const sensor_msgs::ImageConstPtr& msg)
    {
      cv::Mat img_hsv, img_threshold; // Declare variables to store the modified images

      // Get raw image from top camera
      cv_bridge::CvImagePtr cv_ptr_top;
      try
      {
        cv_ptr_top = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      Mat img_color = cv_ptr_top->image;

      cv::imshow(OPENCV_WINDOW_TOP_CAMERA, cv_ptr_top->image); // output of the raw image
      cv::waitKey(3);

      /* Point 2: HSV Image */
    Mat image_hsv;
    Mat image_binary_trackbars;
    cvtColor(img_color, image_hsv, CV_BGR2HSV);
    imshow("HSV Image", image_hsv);

    /*Find proper parameters using trackbars*/
    /* Cases:
        select=0 -> The user could use the other trackbars to set appropiate values for H,S and V
        select=1 -> Default values are used to extract objetc with red color
        select=2 -> Default values are used to extract objetc with green color
        select=3 -> Default values are used to extract objetc with blue color

    */
    sprintf( selectHSV, "select");
    createTrackbar( selectHSV, "HSV Image", &valselect, 3, on_trackbar );

    if (valselect==0){ /*Set up manually*/
        sprintf( tminH, "tminH");
        createTrackbar( tminH, "HSV Image", &minH, slider_max, on_trackbar );
        sprintf( tmaxH, "tmaxH");
        createTrackbar( tmaxH, "HSV Image", &maxH, slider_max, on_trackbar );
        sprintf( tminS, "tminS");
        createTrackbar( tminS, "HSV Image", &minS, slider_max, on_trackbar );
        sprintf( tmaxS, "tmaxS");
        createTrackbar( tmaxS, "HSV Image", &maxS, slider_max, on_trackbar );
        sprintf( tminV, "tminV");
        createTrackbar( tminV, "HSV Image", &minV, slider_max, on_trackbar );
        sprintf( tmaxV, "tmaxV");
        createTrackbar( tmaxV, "HSV Image", &maxV, slider_max, on_trackbar );
     }

     else if(valselect==1){ //Yellow ball light off/on
        minH=0;
        maxH=43;
        minS=162;
        maxS=255;
        minV=169;
        maxV=255;
     }
     else if(valselect==2){ //Red ball light off/on
        minH=0;
        maxH=15;
        minS=77;
        maxS=255;
        minV=192;
        maxV=255;
     }
     else{		 // Blue ball light off/on
        minH=84;//103;
        maxH=110;//64;
        minS=61;//103;
        maxS=255;//255;
        minV=180;//66;
        maxV=255;//130;
     }

     inRange(image_hsv, Scalar(minH,minS,minV), Scalar(maxH,maxS,maxV), image_binary_trackbars);
     imshow("Color extraction", image_binary_trackbars);

      nao_control_tutorial_3::TrackBall srv;
      ros::ServiceClient client = this->nh_.serviceClient<nao_control_tutorial_3::TrackBall>("track_ball");

      nao_control_tutorial_3::GrabBall srv2;
      ros::ServiceClient client2 = this->nh_.serviceClient<nao_control_tutorial_3::GrabBall>("grab_ball");

      nao_control_tutorial_3::HandOverBall srv3;
      ros::ServiceClient client3 = this->nh_.serviceClient<nao_control_tutorial_3::HandOverBall>("handover_ball");

      vector<float> position;

#if 0
      srv.request.color = "yellow";
      if(!client.call(srv))
          {
              ROS_INFO_STREAM("Problem with service TrackBall");
          }
      else
      {
          position = srv.response.pos;
          //cout << position[0] << position[1] << endl;
      }

      srv2.request.pos_ini.clear();
      srv2.request.pos_ini.push_back(position[0]);
      srv2.request.pos_ini.push_back(position[1]);
      srv2.request.pos_ini.push_back(position[2]);
      cout << position[0] << " " << position[1] << " " << position[2] << endl;

#if 0
      if(position[2] != -1)
      {
      if(!client2.call(srv2))
          {
              ROS_INFO_STREAM("Problem with service GrabBall");
          }
      else
      {
          //position = srv.response.pos;
          //cout << position[0] << position[1] << endl;
      }
      }
#endif
#endif

#if 0
       MarkerDetector MDetector;
            vector<Marker> Markers;
            MDetector.detect(img_color,Markers);
            int  sizem= Markers.size();
            //Positions
            float pos_x,pos_y,pos_z;


          for (int i=0; i<sizem; i++) {
                Markers[i].calculateExtrinsics(0.04,cameraP,dist);
                Markers[i].draw(img_color,Scalar(0,0,255),2);
                //3D Coordinates of the marker
                pos_x=Markers[i].Tvec.at<Vec3f>(0,0)[0];
                pos_y=Markers[i].Tvec.at<Vec3f>(0,0)[1];
                pos_z=Markers[i].Tvec.at<Vec3f>(0,0)[2];
                srv2.request.pos_ini.push_back(pos_x);
                srv2.request.pos_ini.push_back(pos_y);
                srv2.request.pos_ini.push_back(pos_z);
                cout << "lo veo" << endl;
                if(!client2.call(srv2))
                {
                    ROS_INFO_STREAM("Problem with second service GrabBall");
                }
                else
                {
                    ROS_INFO_STREAM("All good :), second service being called");
                    //position = srv2.response.pos;
                    //cout << position[0] << position[1] << position[2] << endl;
                }

            }
#
#endif

#if 0
          srv2.request.pos_ini.push_back(0.10); //x
          srv2.request.pos_ini.push_back(0.0);  //y
          srv2.request.pos_ini.push_back(-0.03); //z
          cout << "lo veo" << endl;
          if(!client2.call(srv2))
          {
              ROS_INFO_STREAM("Problem with second service GrabBall");
          }
          else
          {
              ROS_INFO_STREAM("All good :), second service being called");
              Duration(1).sleep();
              //position = srv2.response.pos;
              //cout << position[0] << position[1] << position[2] << endl;
          }
#endif
#if 0
          if(!client3.call(srv3))
          {
              ROS_INFO_STREAM("Problem with third service HandOverBall");
          }
          else
          {
              ROS_INFO_STREAM("All good :), second service being called");
              Duration(10).sleep();
              //position = srv2.response.pos;
              //cout << position[0] << position[1] << position[2] << endl;
          }
#endif

      //srv2.request.pos_ini = position;

      /*if(!client2.call(srv2))
      {
          ROS_INFO_STREAM("Problem with second service GrabBall");
      }
      else
      {
          ROS_INFO_STREAM("All good :), second service being called");
          //position = srv2.response.pos;
          //cout << position[0] << position[1] << position[2] << endl;
      }*/

          //rate_sleep.sleep();
          //Duration(4).sleep();
    }


 /*   void main_loop()
    {
        nao_control_tutorial_3::TrackBall srv;
        ros::ServiceClient client = this->nh_.serviceClient<nao_control_tutorial_3::TrackBall>("track_ball");

        nao_control_tutorial_3::GrabBall srv2;
        ros::ServiceClient client2 = this->nh_.serviceClient<nao_control_tutorial_3::GrabBall>("grab_ball");

        vector<float> position;

        ros::Rate rate_sleep(10);
        if(!client.call(srv))
            {
                ROS_INFO_STREAM("Problem with service TrackBall");
            }
        else
        {
            position = srv.response.pos;
            cout << position[0] << endl;
        }
            rate_sleep.sleep();


        srv2.request.pos_ini.clear();


        srv2.request.pos_ini = position;


        if(!client2.call(srv2))
        {
            ROS_INFO_STREAM("Problem with second service GrabBall");
        }
        else
        {
            ROS_INFO_STREAM("All good :), second service being called");
            //position = srv2.response.pos;
            //cout << position[0] << position[1] << position[2] << endl;
        }
            rate_sleep.sleep();



    }*/

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "francisco");

    ros::NodeHandle n;
    ros::Rate rate_sleep(20);
    Nao_control ic;
    //ic.main_loop();
    ros::spin();
	return 0;

}

