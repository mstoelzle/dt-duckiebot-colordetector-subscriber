#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from opencv import cv2
import numpy as np

class SuscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(SuscriberNode, self).__init__(node_name=node_name)
        # construct publisher
        self.sub = rospy.Subscriber("MaxiColorDetector", String, self.callback)

        self.color = 'red'

    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)
        self.color_detection(data, self.color)

    def color_detection(self, img, color):
        # converting from BGR to HSV color space
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        if color == 'red':
            # Range for lower red
            lower_red = np.array([0,120,70])
            upper_red = np.array([10,255,255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)

            # Range for upper range
            lower_red = np.array([170,120,70])
            upper_red = np.array([180,255,255])
            mask2 = cv2.inRange(hsv,lower_red,upper_red)


        # Generating the final mask to detect red color
        mask = mask1+mask2

        mask1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
        mask1 = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3,3),np.uint8))


        #creating an inverted mask to segment out the cloth from the frame
        mask2 = cv2.bitwise_not(mask1)


        #Segmenting the cloth out of the frame using bitwise and with the inverted mask
        res1 = cv2.bitwise_and(img, img,mask=mask2)

if __name__ == '__main__':
    # create the node
    node = SuscriberNode(node_name='suscriber_node')
    # keep spinning
    rospy.spin()