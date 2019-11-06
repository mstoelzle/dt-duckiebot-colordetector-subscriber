#!/usr/bin/env python

import os
import rospy
import rosbag
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class SubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(SubscriberNode, self).__init__(node_name=node_name)

        self.vehicle_name = rospy.get_param("/vehicle_name")
        self.color = rospy.get_param("/color")

        # construct subscriber
        self.topic = '/' + self.vehicle_name + '/colordetector_publisher_node/image/compressed'
        self.sub = rospy.Subscriber(self.topic, CompressedImage, self.callback)

        self.bridge = CvBridge()

        self.bag_filename = self.topic+'_debug'
        self.bag = rosbag.Bag('/data/'+self.bag_filename+'.bag','w')

    def callback(self, data):
        try:
            print("Received image message")
            rospy.loginfo("I received a message")
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            debug_img = self.color_detection(img, self.color)
            debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img, "jpg")
            self.bag.write(self.topic, debug_msg)
        except Exception as e:
            print(e)

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
    node = SubscriberNode(node_name='subscriber_node')
    # keep spinning
    rospy.spin()