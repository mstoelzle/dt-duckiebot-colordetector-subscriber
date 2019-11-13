#!/usr/bin/env python

import time
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
        self.color = rospy.get_param("~color")

        self.topic_name = 'colordetector_publisher_node'

        # construct subscriber
        self.topic = '/' + self.vehicle_name + '/'+self.topic_name+'/image/compressed'
        self.sub = rospy.Subscriber(self.topic, CompressedImage, self.callback)

        self.bridge = CvBridge()

        # publish and save debug images
        self.debug_topic = '/' + self.vehicle_name + '/colordetector_publisher_node/debug_image_'+self.color+'/compressed'
        self.pub = rospy.Publisher(self.debug_topic, CompressedImage, queue_size=10)

        self.timestamp = time.time()
        self.bag_filename = 'colordetector_subscriber_'+str(self.timestamp)
        self.bag = rosbag.Bag('/data/'+self.bag_filename+'.bag','w')

    def callback(self, msg):
        try:
            rospy.loginfo("I received a message")

            self.bag.write(self.topic, msg)

            raw_img = self.bridge.compressed_imgmsg_to_cv2(msg)

            # cv2.imwrite('/data/test/original.jpg', raw_img)

            debug_img = self.color_detection(raw_img, self.color)

            # cv2.imwrite('/data/test/debug_img_'+self.color+'.jpg', debug_img)

            debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)

            self.pub.publish(debug_msg)
            self.bag.write(self.debug_topic, debug_msg)
            rospy.loginfo("I published a message")
        except Exception as e:
            print(e)

    def color_detection(self, img, color):
        # converting from BGR to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        rospy.loginfo("Converted to hsv color space")

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
        rospy.loginfo("Created mask")

        # The bitwise and of the frame and mask is done so
        # that only the coloured objects are highlighted
        # and stored in res
        res = cv2.bitwise_and(img,img, mask= mask)
        rospy.loginfo("Created res")

        # create debug image
        debug_img = img

        # get object contour
        im_bw = cv2.cvtColor(res, cv2.COLOR_RGB2GRAY)
        thresh, img_bw = cv2.threshold(im_bw, 60, 255, cv2.THRESH_BINARY)
        # contours, hierarchy = cv2.findContours(img_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, contours, _ = cv2.findContours(img_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rospy.loginfo("Found contours")
        cv2.drawContours(res, contours, -1, (0,255,0), 3)
        rospy.loginfo("drew contours")
        for i in range(0,len(contours)):
            x,y,w,h = cv2.boundingRect(contours[i])
            cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0, 255, 0),3)

        rospy.loginfo("drew all bounding rectangles")

        return debug_img

if __name__ == '__main__':
    # create the node
    node = SubscriberNode(node_name='subscriber_node')
    # keep spinning
    rospy.spin()