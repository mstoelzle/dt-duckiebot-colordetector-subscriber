#!/usr/bin/env python

import os
import cv2
import numpy as np
from matplotlib import pyplot as plt
from pprint import pprint

filename ='./Wr2tZ.png'

img = cv2.imread(filename, 1)

# Converts images from BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

lower =  np.array([21, 39, 64])
upper =  np.array([40, 255, 255])
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
    cv2.rectangle(img, (x,y), (x+w,y+h), (0, 255, 0),3)


cv2.imshow('original image',img)
#cv2.imshow('mask',mask)
#cv2.imshow('res',res)
cv2.imshow('debug image',debug_img)

# Destroys all of the HighGUI windows.
key = cv2.waitKey(10000)#pauses for 3 seconds before fetching next image
if key == 27:#if ESC is pressed, exit loop
    cv2.destroyAllWindows()

cv2.imwrite(filename+'_res',res)