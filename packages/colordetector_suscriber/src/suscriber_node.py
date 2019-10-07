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
        rospy.loginfo("I received a message")
        debug_img = self.color_detection(data, self.color)
        

    def color_detection(self, img, color):
        # converting from BGR to HSV color space
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        if color == 'red':
            lower = np.array([155,25,0])
            upper = np.array([179,255,255])
        elif color == 'blue':
            lower = np.array([110,50,50])
            upper = np.array([130,255,255])
        elif color == 'yellow':
            lower =  np.array([21, 39, 64])
            upper =  np.array([40, 255, 255])
        else:
            print('unknown color')
            return


        mask = cv2.inRange(hsv, lower, upper)

        # The bitwise and of the frame and mask is done so
        # that only the blue coloured objects are highlighted
        # and stored in res
        res = cv2.bitwise_and(img,img, mask= mask)

        # create debug image
        debug_img = img

        # get object contour
        im_bw = cv2.cvtColor(res, cv2.COLOR_RGB2GRAY)
        thresh, img_bw = cv2.threshold(im_bw, 60, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(img_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(res, contours, -1, (0,255,0), 3)
        for i in range(0,len(contours)):
            x,y,w,h = cv2.boundingRect(contours[i])
            cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0, 255, 0),3)

        return debug_img

if __name__ == '__main__':
    # create the node
    node = SuscriberNode(node_name='suscriber_node')
    # keep spinning
    rospy.spin()