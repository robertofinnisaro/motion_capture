#!/usr/bin/env python3

from sys import _current_frames
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(data):
    br = CvBridge()
    rospy.loginfo("recieveing video frame")
    current_frame = br.imgmsg_to_cv2(data)
    cv2.imshow("camera", current_frame)
    cv2.waitKey(1)

def recieve_message():
    rospy.init_node('video_sub_py', anonymous=True)
    # rospy.Subscriber('/camera_1/camera_1/image_raw', Image, callback)
    rospy.Subscriber('video_frames', Image, callback)
    rospy.spin()
    cv2.destoryAllWindows()

if __name__ == '__main__':
    recieve_message()   