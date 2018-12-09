#!/usr/bin/env python
#!*******************************************************************************************************************************/
#!                                                                                                                              */
#! Date: 18.12.2017                                                                                                             */
#! Author: German Diez Valencia    germandiezvalencia@gmail.com                                                                 */
#! based on the public scikit-learn example http://scikit-learn.org/0.18/auto_examples/applications/face_recognition.html       */
#! Group A                                                                                                                      */
#! Face recognition module:                                                                                                                  */
#! Final project Humanoid Robotic Systems                                                                                                     */
#!*******************************************************************************************************************************/
import rospy
import time
import motion
import numpy as np
import almath
import sys
from naoqi import ALProxy
from tutorial_6.srv import RoiFace
from tutorial_6.srv import RoiFaceResponse
import motion

from tf import TransformListener
from tf import transformations

import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

from time import time
import logging
import matplotlib.pyplot as plt

from sklearn.model_selection import train_test_split
from sklearn.model_selection import GridSearchCV
from sklearn.datasets import fetch_lfw_people
from sklearn.metrics import classification_report
from sklearn.metrics import confusion_matrix
from sklearn.decomposition import PCA
from sklearn.svm import SVC
import pandas
import pickle

import os

motionProxy =0;

		
class FaceDetection:
    def __init__(self):
        image_sub = rospy.Subscriber('/nao_robot/camera/top/camera/image_raw',Image, self.imagecallback)
        s = rospy.Service('face_service', RoiFace, self.sendPositionNAO)
        rospy.init_node("face_reco_service")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/nao_robot/camera/top/camera/image_raw',Image,self.imagecallback)
        clf_name="/home/hrs_a/catkin_ws/src/tutorial_6/clf.sav"
        pca_name="/home/hrs_a/catkin_ws/src/tutorial_6/pca.sav"
        self.svm_ready=False
        self.h=50
        self.w=37
        self.cnt_samples=0
        self.cnt_classes=0
        self.X=[] #features Vector
        self.y=[] #lables vector
        self.target_names=[] #class name
        self.faces_folder=os.listdir("/home/hrs_a/catkin_ws/src/tutorial_6/faceDB")
        try:
            self.clf=pickle.load(open(clf_name,'rb'))
            self.pca=pickle.load(open(pca_name,'rb'))
            for ind in self.faces_folder:
                self.target_names.append(ind)
                print("checking the names",ind)
            print("Model loaded directly")
        except:

            #-------------SVM

            # get images to train
            for ind in self.faces_folder:
                print("debug:",self.cnt_classes,ind,self.target_names)
                self.target_names.append(ind)
                path="/home/hrs_a/catkin_ws/src/tutorial_6/faceDB/"+ind+"/"
                indiv=os.listdir(path)
                self.cnt_classes+=1
                for face in indiv:
                    image = cv2.imread(path+face,0)
                    image = cv2.resize(image,(self.h,self.w))
                    self.X.append(image.reshape((self.h*self.w)))
                    self.y.append(self.cnt_classes)
                    self.cnt_samples+=1
            self.X=np.array(self.X)
            self.y=np.array(self.y)
            self.target_names=np.array(self.target_names)
            self.n_samples=self.cnt_samples
            self.n_features = self.X.shape[1]
            self.n_classes = self.target_names.shape[0]

            print("Total dataset size:")
            print("n_samples: %d" % self.n_samples)
            print("n_features: %d" % self.n_features)
            print("n_classes: %d" % self.n_classes)


            ###############################################################################
            # Split into a training set and a test set using a stratified k fold

            # split into a training and testing set
            self.X_train, self.X_test, self.y_train, self.y_test = train_test_split(
                self.X, self.y, test_size=0.25, random_state=42)


            ###############################################################################
            # Compute a PCA (eigenfaces) on the face dataset (treated as unlabeled
            # dataset): unsupervised feature extraction / dimensionality reduction
            self.n_components = 150

            print("Extracting the top %d eigenfaces from %d faces"
                  % (self.n_components, self.X_train.shape[0]))

            self.pca = PCA(n_components=self.n_components, svd_solver='randomized',
                      whiten=True).fit(self.X_train)


            self.eigenfaces = self.pca.components_.reshape((self.n_components, self.h, self.w))

            print("Projecting the input data on the eigenfaces orthonormal basis")
            #t0 = time()
            self.X_train_pca = self.pca.transform(self.X_train)
            self.X_test_pca = self.pca.transform(self.X_test)
            #print("done in %0.3fs" % (time() - t0))


            ###############################################################################
            # Train a SVM classification model

            print("Fitting the classifier to the training set")
            #t0 = time()
            self.param_grid = {'C': [1e3, 5e3, 1e4, 5e4, 1e5],
                          'gamma': [0.0001, 0.0005, 0.001, 0.005, 0.01, 0.1], }
            self.clf = GridSearchCV(SVC(kernel='rbf', class_weight='balanced'), self.param_grid)
            self.clf = self.clf.fit(self.X_train_pca, self.y_train)
            #print("done in %0.3fs" % (time() - t0))
            print("Best estimator found by grid search:")
            print(self.clf.best_estimator_)


            ###############################################################################
            # Quantitative evaluation of the model quality on the test set

            print("Predicting people's names on the test set")
            #t0 = time()
            self.y_pred = self.clf.predict(self.X_test_pca)

            pickle.dump(self.clf,open(clf_name,"wb"))
            pickle.dump(self.pca,open(pca_name,"wb"))
            print(classification_report(self.y_test, self.y_pred, target_names=self.target_names))
            print(confusion_matrix(self.y_test, self.y_pred, labels=range(self.n_classes)))

        self.svm_ready=True

    def predict(self, image):#predictions
        image1=cv2.resize(image,(self.h,self.w))
        imgerz=image1.reshape((self.h*self.w))
        reshape=[imgerz]
        reshape=np.array(reshape)
        imgerz_pca=self.pca.transform(reshape)
        self.out=self.clf.predict(imgerz_pca)
        self.face_name=self.target_names[self.out[0]-1]
        return self.face_name

    def sendPositionNAO(self,req): #recieve the rect and makes a prediction
        gray=cv2.cvtColor( self.cv_im, cv2.COLOR_RGB2GRAY )
        labels = [None] * len(req.y)
        for i in range(len(req.y)):
            crop=gray[req.y[i]:req.y[i]+req.h[i], req.x[i]:req.x[i]+req.w[i]]
            cv2.imshow("SVM IMG", self.cv_im)
            if self.svm_ready:
                person=self.predict(crop)
                labels[i]=person
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.rectangle(self.cv_im,(req.x[i],req.y[i]),(req.x[i]+req.w[i],req.y[i]+req.h[i]),(0,255,0),3)
                cv2.putText(self.cv_im,person,(req.x[i]+req.w[i],req.y[i]+req.h[i]), cv2.FONT_HERSHEY_SIMPLEX, 4, 255)

        cv2.imshow("SVM IMG", self.cv_im)
        cv2.waitKey(1)
        print(labels)
        return RoiFaceResponse(labels)
    def imagecallback(self,image):
        debug = False
        try:
            self.cv_im = self.bridge.imgmsg_to_cv2(image,"bgr8")
        except CvBridgeError, e:
            data_str = "CvBridge: {}".format(e);
            rospy.loginfo(data_str)
            return
        im_gray = cv2.cvtColor(self.cv_im, cv2.COLOR_BGR2GRAY)

if __name__ == '__main__':

    FD = FaceDetection()
    try:
            rospy.spin()
    except rospy.ROSInterruptException:
            pass
    rospy.loginfo("FaceDetection shut down")
    cv2.destroyAllWindows()
