#!/usr/bin/env python3
#Initialization of Libraries
import rospy
#for server-client communication (request,reply)
from my_robot_tutorial.srv import facedetect, facedetectResponse
import numpy as np
import os
import cv2  #OpenCV Library
from cv_bridge import CvBridge # provides an interface between ROS and OpenCV
import sys
#Class definition
class facedetectionclass:

    def __init__(self):
        #options for available images
        self.availableimages = [1, 2, 3, 4, 5, 6]
        #creating a ROS Service named as 'facedetect'
        self.ros_service = rospy.Service("facedetect", facedetect, self.sendimage)
    #Defining a function for image acquisition from the folder
    def readbyfilename(self, filename):
        directoryname = os.path.dirname(__file__)
        filelocation = directoryname + "/Images/" + filename
        #reading image
        image = cv2.imread(filelocation)
        return image
    #returns the value out of range to maximum limit of available range
    def getimage(self,img):
        chooseimage = min(self.availableimages, key = lambda x:abs(x-img))
        #Creating an image name from the user input
        return self.readbyfilename(str(chooseimage) + '.jpg')
    #returns image with faces detected
    def sendimage(self, req):
        image = self.getimage(req.face_image)
        #complete path to pre-trained classifier file
        cascpath = "/home/wania/catkin_ws/src/my_robot_tutorial/scripts/haarcascade_frontalface_default.xml"
        # Model preparation for face detection
        faceCascade = cv2.CascadeClassifier(cascpath)
        #conversion from colored image to gray scale image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # model implementation on gray-scaled image with parameters to adjust as requirement
        faces = faceCascade.detectMultiScale(gray, scaleFactor = 1.18, minNeighbors = 5, flags = cv2.CASCADE_SCALE_IMAGE)
        #print the number of faces detected in the image
        #print("Found {0} faces in the picture!".format(len(faces)))
        # placing blue rectangles on detected faces
        for (x,y,w,h) in faces:
            cv2.rectangle(image,(x,y), (x+w, y+h), (255, 0,0),2)
        # converting image to image message type for a reply to client
        image_msg = CvBridge().cv2_to_imgmsg(image)
        # reply to client's request
        return facedetectResponse(image_msg)
#main function of the script
if __name__ == '__main__':
    try:
        #Initializtion of server node
        rospy.init_node("face_detect_server_node")
        #calling the class
        facedetectionclass()
        print ("Service is running")
        rospy.spin()
    #to handle exception occur during running process
    except rospy.ROSInterruptException as e :
        print(e)
