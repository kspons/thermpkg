#! /usr/bin/env python2

import sys
import rospy
import cv2
import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int16MultiArray

def simple_img(tempData): # outputs a viewable image
    # input is 2D numpy array of temperature data
    # normalize the pixels between 0 and 255 to make it viewable
    temp_min = tempData.min()
    temp_max = tempData.max()
    temp_range = float(temp_max - temp_min)
    bwimg = (tempData - temp_min) / temp_range * 255
    bwimg = bwimg.astype('uint8')
    # Add approximate min/max temperature seen in the image (deg C)
    # cv2.putText(bwimg, str(tempData[0,0]),
    # (0,20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))
    # cv2.putText(bwimg, str(temperature_estimator(temp_min)),
    # (0,280), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))
    # cv2.circle(bwimg, (100,100), 2, (255,255,255), 1)
    # cv2.putText(bwimg, str(tempData[100,100]),
    # (100,100), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))
    nppoints = [(100,100), (200,90), (200,280), (203,10), (280,300), (73,200)]
    cvpoints = [(100,100), (90,200), (280,200), (10,203), (300,280), (208,10)]
    for i in range(len(nppoints)):
        cv2.circle(bwimg, cvpoints[i], 1, (255,255,255), 1)
        # cv2.putText(bwimg, str(tempData[points[i]]),
        # points[i], cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))
        # "temp"
        cv2.putText(bwimg, str(temperature_estimator(tempData[nppoints[i]])),
        cvpoints[i], cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))

    return bwimg

def temperature_estimator(input):
    # converts ThermApp data point into a 'real' temperature (deg C)
    out = (input+275)
    # out = 0.2817*input + 46.127
    # thermapp uses (t - 14336) * 0.00652
    return out

def simpleImgCallback(temp_msg):
    # turn the ROS message into a numpy array
    # tempData = cv2.flip(np.reshape(temp_msg.data, (288,384)), 0)
    tempData = np.reshape(temp_msg.data, (288,384))
    # generate simple image for viewing
    min_img = simple_img(tempData)
    # display image
    cv2.imshow('Thermal Image', min_img)
    waitkey = cv2.waitKey(1)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')

def temp_subscriber():
    rospy.init_node('simple_img')
    rospy.Subscriber("temperature", Int16MultiArray, simpleImgCallback)
    rospy.spin()

def myshutdown():
    print('shutting down, see ya')
    cv2.destroyAllWindows

if __name__ == '__main__':
    temp_subscriber()