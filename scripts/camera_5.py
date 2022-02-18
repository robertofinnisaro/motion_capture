#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera5:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera_5/camera_5/image_raw", Image, self.callback)

    def callback(self,data):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        image4 = cv_image

        resized_image = cv2.resize(image4, (360, 640)) 

        cv2.imshow("Camera output 5", image4)
        #cv2.imshow("Camera output resized", resized_image)

        cv2.waitKey(3)


def main():
    Camera5()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_5', anonymous=False)
    main()