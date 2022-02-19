#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera3:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera_3/camera_3/image_raw", Image, self.callback)

    def callback(self,data):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        image3 = cv_image

        resized_image = cv2.resize(image3, (360, 640)) 

        cv2.moveWindow("Camera output 3", 20, 300)
        cv2.imshow("Camera output 3", image3)
        #cv2.imshow("Camera output resized", resized_image)

        cv2.waitKey(3)


def main():
    Camera3()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_3', anonymous=False)
    main()