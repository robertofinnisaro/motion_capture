#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera2:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera_2/camera_2/image_raw", Image, self.callback)

    def callback(self,data):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        image2 = cv_image

        resized_image = cv2.resize(image2, (360, 640)) 

        cv2.moveWindow("Camera output 2", 340, 20)
        cv2.imshow("Camera output 2", image2)
        #cv2.imshow("Camera output resized", resized_image)

        cv2.waitKey(3)


def main():
    Camera2()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_2', anonymous=False)
    main()