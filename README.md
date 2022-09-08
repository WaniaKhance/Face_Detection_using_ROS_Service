# Face Detection using ROS Services

## Server and Client Communication - Intelligent Systems and Robotics

* Introduction
  ------------

In this project, we described a system which is capable of detecting faces in the images. This system design is based on server-client communication that requires request / reply interaction. To implement this system, we are using ROS Service, which is defined by a pair of messages: one for the request and one for the reply. The server node offers a service for face detection under a name-string type ‘facedetect’, and a client node calls the service by sending the request message by choosing an image and waits for the reply. 


* Python Files Description
  ------------

We have developed this system in ROS using server, client nodes and service request/reply method where:

1.	Face_dectect_server.py: initiates a new service for the client. On receiving the client’s request, image will be acquired from its corresponding folder and applied to HAAR Cascade classifier for face detection process. Response to the client’s request will be made using an image type message from sensor_msgs package.

2.	Face_dectect_client.py: make a request by choosing a number for image selection, which will be sent to server for image acquisition and detection process. 

3.	Facedetect.srv: Message type will be defined here for request/reply communication between server and client.


* Requirements
  ------------

1.	Linux OS - Ubuntu 20.04 

2.	Python 3.7 or above 


* Installation
  ------------
  
First, we need to install few libraries in order to run these files.

1.	Installation of OpenCV: sudo apt-get install python3-opencv
2.	Download haarcascade_frontalface_default.xml file from the github depository in the same folder where the project is locating. (file given in the zip folder)
3.	Create a folder as ‘Image’ inside the same directory and add sample images.
4.	Add service file name i.e. facedetect.srv in the srv generate folder of CMakeLists.txt file. 


* Output Results
  ------------
 
To run python source codes, we first source our setup.bash file and execute the server file. Once the service starts running, we can see from rosservice list. To request this service, we will execute the client file as given below:

![alt text](https://github.com/WaniaKhance/Face_Detection_using_ROS_Services/blob/main/Picture4.png?raw=true)


