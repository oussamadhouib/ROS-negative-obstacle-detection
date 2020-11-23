#!/usr/bin/env python

#library to use in our model
import time
from keras.datasets import mnist
from keras.models import Sequential
from keras.models import model_from_json
from keras.models import load_model
from keras.layers import Dense
from keras.layers import Dropout
from keras.utils import np_utils
import imutils
import numpy as np
from sklearn.metrics import pairwise
import glob
from std_msgs.msg import Float32
from std_msgs.msg import String

#opencv
import cv2

import rospy
#Import bridge to convert open cv frames into ros frames
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

global loadedModel
size = 300

# resize the frame to required dimensions and predict
def predict_pothole(currentFrame):

    currentFrame = cv2.resize(currentFrame,(size,size))
    currentFrame = currentFrame.reshape(1,size,size,1).astype('float')
    currentFrame = currentFrame/255
    prob = loadedModel.predict_proba(currentFrame)
    max_prob = max(prob[0])
    if(max_prob>.90):
        return loadedModel.predict_classes(currentFrame) , max_prob
    return "none",0

## import model
loadedModel = load_model('/home/oussama/Desktop/CNN-potholes-detection/pothole-detection-system-using-convolution-neural-networks/Real-time Files/model_detect.h5')
# declaration show_pred
show_pred = False
# Capture frames from a camera
camera = cv2.VideoCapture('rtsp://root:En0vaR0b0tics@192.168.1.108:1025/live4.sdp')
# loop runs if capturing has been initialized.
while 1: 
        #Initializing publisher
        pub = rospy.Publisher("frames", Image, queue_size=10)
        pub1 = rospy.Publisher('detect_negative', Float32, queue_size = 1)
        rospy.init_node('stream_publisher', anonymous=True)
        rate = rospy.Rate(10)

        #msg_string = String()

        # Reads frames from a camera
        (grabbed,frame) = camera.read()
        frame = imutils.resize(frame,width = 700)
        frame = cv2.flip(frame,1)
        clone = frame.copy()
        (height,width) = frame.shape[:2]
        # Convert to gray scale of each frames
        grayClone = cv2.cvtColor(clone,cv2.COLOR_BGR2GRAY)

        # Detects negative obstacle in the input image
        pothole,prob = predict_pothole(grayClone)
        keypress_toshow = cv2.waitKey(1)

        #Publish int (0 or 1)
        #msg_float.data = float()
        #pub1.publish(float(pothole))



        # Show results (just enter 'e' to see predection)
        if(keypress_toshow == ord("e")):
            show_pred = not show_pred
        
        if(show_pred):
            cv2.putText(clone , str(pothole)+' '+str(prob*100)+'%' , (30,30) , cv2.FONT_HERSHEY_DUPLEX , 1 , (0,255,0) , 1)

        #cv2.imshow("GrayClone",grayClone) 

        #display an image in a window en realtime
        cv2.imshow("Video Feed",clone)
            #Give the frames to ros Environment 
        bridge = CvBridge() 
            #Encoding bgr8: CV_8UC3 color image with blue-green-red color order    
        ros_image = bridge.cv2_to_imgmsg(clone, "bgr8")
        pub.publish(ros_image)
        rate.sleep()

        keypress = cv2.waitKey(1) & 0xFF
        if(keypress == ord("q")):
            break

# Close the window
camera.release()

# De-allocate any associated memory usage
cv2.destroyAllWindows() 
