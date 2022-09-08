#!/usr/bin/env python3
#same as mentioned for the server node
import rospy
from my_robot_tutorial.srv import facedetect, facedetectResponse
import numpy as np
import os
import cv2
from cv_bridge import CvBridge
import sys
#function definition to request for the service and image display
def configurereq(image):
    #waits till service is available
    rospy.wait_for_service("facedetect")
    try:
        #calling a service
        service_proxy = rospy.ServiceProxy("facedetect", facedetect)
        #service response as an image msg
        response_msg = service_proxy(image)
        image_msg = response_msg.image
        #converting image msg to an image with the same encoding as image msg
        image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding = "passthrough")
        print(" Face Detected ")
        #displaying image
        cv2.imshow("Face Detection Complete",image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    #if a service returns an error for the request
    except rospy.ServiceException as e:
        print("Service Failed")
        print(e)


if __name__ == '__main__':
    try:
        #Initializtion of client node
        rospy.init_node("face_detect_client_node")
        userinput = input("\n Enter number between 1 to 6 to choose image for face detection: ")
        #infinite loop until user types q for quit
        while userinput != "q":
            try:
                #calling function with a user input as a parameter
                configurereq(float(userinput))
                userinput = input("\n Enter number between 1 to 6 to choose image for face detection: ")
            #if any exception occurs
            except Exception as e:
                print ("Error running service")
                print (e)

    except rospy.ROSInterruptException:
        pass
