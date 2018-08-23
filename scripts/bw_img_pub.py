#! /usr/bin/env python2

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("temperature", Int16MultiArray, self.callback)
    
    def callback(self, temp_msg):
        tempData = np.reshape(temp_msg.data, (288,384))
        temp_min = tempData.min()
        temp_max = tempData.max()
        temp_range = float(temp_max - temp_min)
        bwimg = (tempData - temp_min) / temp_range * 255
        cv_image = bwimg.astype('uint8')
        flip = True
        if flip:
            cv_image = cv2.flip(cv_image, 0)
        cv2.imshow('Thermal Image', cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
        except CvBridgeError as e:
            print(e)

def temp_subscriber():
    ic = image_converter()
    rospy.init_node('listener')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    temp_subscriber()
