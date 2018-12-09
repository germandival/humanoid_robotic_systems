/**************************************************************************/
/*                                                                        */
/* Date: 10.01.2018                                                       */
/* Author: German Diez Valencia    germandiezvalencia@gmail.com           */
/*                                                                        */
/* IndiaNAO johnes                                                        */
/* Humanoid Robotic Systems                                               */
/* this code is a modification of the  OpenCv tutorial for face detaction */
/* using haar cascade classifiers, this tutorial can be found in the link:*/ 
/* https://docs.opencv.org/2.4/doc/tutorials/objdetect/cascade_classifier/cascade_classifier.html                                                   */
/**************************************************************************/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "tutorial_6/RoiFace.h"
#include "tutorial_6/NaoFace.h"
#include <string.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <image_transport/image_transport.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"

#include <string.h>
#include <math.h>

using namespace std;
using namespace aruco;
using namespace cv;

/** Global variables */
String face_cascade_name = "/home/hrs_a/catkin_ws/src/tutorial_6/haarcascade_frontalface_alt.xml";

CascadeClassifier face_cascade;
vector<Rect> ROIS;
Mat cameraP(3,3,CV_32FC1);
Mat dist(4,1,CV_32FC1);
String nameSave="None";
String DB_path="/home/hrs_a/catkin_ws/src/tutorial_6/faceDB/"+nameSave+"/";
int saveIndex=0;
class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;

    ros::ServiceClient client;

    //aruco camera parameters
    CameraParameters TheCameraParameters;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::ServiceServer service = nh_.advertiseService("/recognize_face", &Nao_control::faceSrvCallBack, this);
    Mat image;
    //camera matrix

    Nao_control():it_(nh_)
    {

        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &Nao_control::imageCb, this);

        client = nh_.serviceClient<tutorial_6::RoiFace>("face_service");
        //distortion matrix of bottom camera

        dist.at<float>(0,0)=-0.0648763971625288;
        dist.at<float>(1,0)=0.0612520196884308;
        dist.at<float>(2,0)=0.0038281538281731;
        dist.at<float>(3,0)=-0.00551104078371959;


        cameraP.at<float>(0,0)=558.570339530768;
        cameraP.at<float>(0,1)=0.000000;
        cameraP.at<float>(0,2)=308.885375457296;
        cameraP.at<float>(1,0)=0.000000;
        cameraP.at<float>(1,1)=556.122943034837;
        cameraP.at<float>(1,2)=247.600724811385;
        cameraP.at<float>(2,0)=0.000000;
        cameraP.at<float>(2,1)=0.000000;
        cameraP.at<float>(2,2)=1.000000;

        TheCameraParameters.setParams(cameraP,dist,Size(640,480));


    }
    ~Nao_control()
    {
    }
    //callback function that recieves the images and usefull to create a face database in the folder DB_path
    void imageCb(const sensor_msgs::ImageConstPtr& img_raw)
    {
        //create a bridge between ros Images and opencv images
        cv_bridge::CvImagePtr  cv_img_ptr;
        Mat gray_img;
        try{
           cv_img_ptr = cv_bridge::toCvCopy(img_raw,"bgr8");
           cv_img_ptr->image.copyTo(image);

        }
        catch (cv_bridge::Exception& e){
           ROS_ERROR("error loading the image");
           return;
        }

        if( !face_cascade.load( face_cascade_name ) ){
            ROS_ERROR("error loading face");
        }
        if(nameSave!="None")
        {
           //detect the faces in the image and set a ROI in the region of the face
           cv::cvtColor(image,gray_img, CV_BGR2GRAY);
           //call the Haarcascade classifier
           face_cascade.detectMultiScale( gray_img, ROIS, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50) );
           // if one face was detected save the image into de database
           for( size_t i = 0; i < ROIS.size(); i++ )
           {
                Rect roi=ROIS[i];
                cv::Mat crop = image(roi);
                cv::imshow("crop", crop);
                string savePath=DB_path+std::to_string(saveIndex)+".png";
                imwrite( savePath, crop );
                saveIndex++;
           }
           cv::imshow("faces XD", image);
           cv::waitKey(1);
        }
    }
    // Callback function for detecting the faces present in the Mat image
    bool faceSrvCallBack(tutorial_6::NaoFace::Request &req, tutorial_6::NaoFace::Response &res)
    {
        // name that the planer rwquest to the service
        string person=req.name;

        ROS_INFO_STREAM(person);
        Mat gray_img;
        cv::cvtColor(image,gray_img, CV_BGR2GRAY);
        //detect the faces in the image and set a ROI in the region of the face
        //call the Haarcascade classifier
        face_cascade.detectMultiScale( gray_img, ROIS, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50) );
        tutorial_6::RoiFace srv;
        //for all the ROIS with faces in the image append a rect in the face classification service request
        for( size_t i = 0; i < ROIS.size(); i++ )
        {
             Rect roi=ROIS[i];
             cv::Mat crop = image(roi);
             //faces.push_back(crop);
             cv::imshow("crop", crop);

             srv.request.x.push_back(roi.x);
             srv.request.y.push_back(roi.y);
             srv.request.h.push_back(roi.height);
             srv.request.w.push_back(roi.width);

        }
        if(ROIS.size()>0)
        {
            //sevice call
            client.call(srv);
            ROS_INFO_STREAM("I was asked for: "<<person);
            for (std::vector<string>::const_iterator i = srv.response.name.begin(); i != srv.response.name.end(); ++i)
            {
            	//analize the response of the service 
                string aux=*i;
                ROS_INFO_STREAM("I recognized: "<<aux);
                //if the classification recognize the seme person requested
                if(aux==person)
                {
                    ROS_INFO_STREAM("I saw the same person");
                    //Look for the aruco of the recognized person 
                    MarkerDetector MDetector;
                    vector<Marker> Markers; /*List with corners of the detected marker*/

                    MDetector.detect(image,Markers); /*Image is where the markers are going to be detected */

                    int  sizem= Markers.size();
                    ROS_WARN_STREAM("sizem: " << sizem);
                    if(sizem==1)
                    {

                        Markers[0].calculateExtrinsics(0.04,cameraP,dist); /* Calculate the rotation (rvec) and translation vectors (tvec)*/

                        res.x=Markers[0].Tvec.at<Vec3f>(0,0)[0];
                        res.y=Markers[0].Tvec.at<Vec3f>(0,0)[1];
                        res.z=Markers[0].Tvec.at<Vec3f>(0,0)[2];
                        res.tetha=Markers[0].Rvec.at<Vec3f>(0,0)[3];
                        res.sci=Markers[0].Rvec.at<Vec3f>(0,0)[4];
                        res.phi=Markers[0].Rvec.at<Vec3f>(0,0)[5];

                    }
                    // return the position of the recognized person
                    res.inImage=true;
                    res.personInImage=aux;
                }
                //in this part somene different to the requested is recognized
                res.personInImage=aux;
                return true;
            }
        }
        //return no person was recognized 
        res.inImage=false;
        res.personInImage="No_one";
        cv::imshow("faces XD", image);
        cv::waitKey(1);
        return true;
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_recognition_node");
    Nao_control ic;
    //ic.sendManual();
    ros::spin();
    return 0;

}
