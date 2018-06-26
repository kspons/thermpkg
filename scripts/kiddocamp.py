#! /usr/bin/env python2

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16MultiArray

def therm_callback(msg):
    thm_img = simple_img(np.reshape(msg.data, (288,384))+800)
    # thm_img = cv2.GaussianBlur(thm_img, (1,1), 0.5)
    heatmap(thm_img)

def heatmap(gryimg):
    global current_map
    # apply color map
    colorimg = cv2.applyColorMap(gryimg, current_map.map)
    # display color mapped image
    cv2.imshow('heat mapped thermal img', colorimg)
    waitkey = cv2.waitKey(1)

    # change color map with arrow keys, exit with esc or q keys
    if waitkey == 27 or waitkey == 113:
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')
    elif waitkey == 82 or waitkey == 83:
        #color map up when up or right arrows are pressed
        current_map.changemap(1)
    elif waitkey == 84 or waitkey == 81:
        #color map down when down or left arrows are pressed
        current_map.changemap(0)
    # elif waitkey == 104:
    #     #make matplotlib histogram when h is pressed
    #     matplothist(gryimg)

def simple_img(tempData): # outputs a viewable image
    # input is 2D numpy array of temperature data
    # normalize the pixels between 0 and 255 to make it viewable
    temp_min = tempData.min()
    temp_max = tempData.max()
    temp_range = float(temp_max - temp_min)
    bwimg = (tempData - temp_min) / temp_range * 255
    bwimg = bwimg.astype('uint8')
    return bwimg

class Colormap():
    maplist = [cv2.COLORMAP_AUTUMN, cv2.COLORMAP_BONE, cv2.COLORMAP_COOL, cv2.COLORMAP_HOT,
              cv2.COLORMAP_HSV, cv2.COLORMAP_JET, cv2.COLORMAP_OCEAN, cv2.COLORMAP_PARULA,
              cv2.COLORMAP_PINK, cv2.COLORMAP_RAINBOW, cv2.COLORMAP_SPRING,
              cv2.COLORMAP_SUMMER, cv2.COLORMAP_WINTER]
    mapstring = ['AUTUMN', 'BONE', 'COOL', 'HOT', 'HSV', 'JET', 'OCEAN', 'PARULA',
                 'PINK', 'RAINBOW', 'SPRING', 'SUMMER', 'WINTER']

    def __init__(self):
        self.index = 1
        self.setmap()

    def setmap(self):
        self.map = self.maplist[self.index]
        self.name = self.mapstring[self.index]

    def changemap(self, updown):
        if updown == 1:
            if self.index == len(self.maplist) - 1:
                self.index = 0
            else:
                self.index += 1
        else:
            if self.index == 0:
                self.index = len(self.maplist) - 1
            else:
                self.index -= 1
        self.setmap()

def myshutdown():
    print('shutting down')
    cv2.destroyAllWindows()

def main():
    rospy.init_node('horizon')
    rospy.Subscriber("/temperature", Int16MultiArray, therm_callback)
    rospy.spin()
    rospy.on_shutdown(myshutdown)

if __name__ == '__main__':
    bridge = CvBridge()
    current_map = Colormap()
    main()