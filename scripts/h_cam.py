#!/usr/bin/env python3

from threading import currentThread
import rospy
import homography as h
import numpy as np
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

br = CvBridge()

def callCam1(data):
    cam1_frame = br.imgmsg_to_cv2(data, "bgr8")
    cv2.imwrite('/home/rob/catkin_ws/src/motion_capture/images/cam1.jpeg', cam1_frame)


def callCam2(data):
    cam2_frame = br.imgmsg_to_cv2(data, "bgr8")
    cv2.imwrite('/home/rob/catkin_ws/src/motion_capture/images/cam2.jpeg', cam2_frame)

def subscribers():
    rospy.init_node('video_sub_py', anonymous=True)
    cam1_image = rospy.Subscriber('/camera_1/camera_1/image_raw', Image, callCam1)
    cam2_image = rospy.Subscriber('/camera_2/camera_2/image_raw', Image, callCam2)
    cv2.waitKey(1)


subscribers()

cam1 = cv2.imread('/home/rob/catkin_ws/src/motion_capture/images/cam1.jpeg')
cam2 = cv2.imread('/home/rob/catkin_ws/src/motion_capture/images/cam2.jpeg')
   
# cam2 = cv2.resize(cam2, None, fx=0.5, fy=0.3)
cam2 = cv2.resize(cam2, (320, 240))
# Ensure that the two images are the same size
cam1 = cv2.resize(cam1, (cam2.shape[1], cam2.shape[0]))

kpcam2, kp1, des1 = h.sift_kp(cam2)
kpcam1, kp2, des2 = h.sift_kp(cam1)

# Display the original image and the image after key point detection at the same time
h.cvshow('cam1', np.hstack((cam1, kpcam1)))
h.cvshow('cam2', np.hstack((cam2, kpcam2)))
goodMatch = h.get_good_match(des1, des2)
all_goodmatch_img = cv2.drawMatches(cam2, kp1, cam1, kp2, goodMatch, None, flags=2)

# goodmatch_img Set the first goodMatch[:10]
goodmatch_img = cv2.drawMatches(cam2, kp1, cam1, kp2, goodMatch[:10], None, flags=2)

h.cvshow('Keypoint Matches1', all_goodmatch_img)
h.cvshow('Keypoint Matches2', goodmatch_img)

# Stitch the picture into a panorama
result = h.siftimg_rightlignment(cam2, cam1)
h.cvshow('result', result)