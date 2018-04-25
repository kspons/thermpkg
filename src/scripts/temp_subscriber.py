#! /usr/bin/env python2

import sys
import rospy
import cv2
import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int16MultiArray

PIXEL_DATA_SIZE = int(384*288)

# Calibration variables
calib_counter = 0
pix_calib = np.zeros(PIXEL_DATA_SIZE)
# deadpixel_map = np.zeros(PIXEL_DATA_SIZE)
deadpixel_map = [0]


def temperature_process(temperature_data):
    
    # turn ROS message into a numpy array
    tempData = np.asarray(temperature_data.data)

    # calibration routine
    global calib_counter
    global pix_calib
    global deadpixel_map
    calib_frames = 20
    if calib_counter < calib_frames:
        # for each pixel, get the average value over x frames
        pix_calib = np.add(pix_calib, tempData)
        print "Obtained Calibration image #", calib_counter+1
        calib_counter += 1

    elif calib_counter == calib_frames:
        pix_calib = pix_calib / (calib_frames)
        meanpixel = np.mean(pix_calib)
        for i in range(0, PIXEL_DATA_SIZE):
            if (pix_calib[i] > meanpixel + 240) or (pix_calib[i] < meanpixel - 240):
                print ("dead pixel at location {0}  ({1} vs {2})".format(i, pix_calib[i], meanpixel))
                deadpixel_map = np.append(deadpixel_map, [i])
        calib_counter += 1
        print ("dead pixels identified at locations:")
        print np.size(deadpixel_map)-1, ' dead pixels detected'
        print "Calibration finished"
    else:
        # replace dead pixels with the pixel value of its neighboor (lazy, i know)
        for i in range(1, len(deadpixel_map)):
            tempData[deadpixel_map[i]] = tempData[deadpixel_map[i]-1]
        calib_gain = 2

        # Temperature data into human viewable image
        scaling_array = np.subtract(tempData, pix_calib)*calib_gain
        temp_min = np.ndarray.min(scaling_array)
        temp_max = np.ndarray.max(scaling_array)
        # temp_min = -100
        # temp_max = 100
        temp_range = np.float(temp_max - temp_min)


        floats = np.array((scaling_array - temp_min ) / (temp_range), dtype=float)
        img = np.array(floats * 255, dtype = np.uint8)
        # now put the pixels into a opencv image compatible numpy array
        cvimg = np.zeros(PIXEL_DATA_SIZE, dtype=np.uint8)
        for i in range (0, PIXEL_DATA_SIZE):
            cvimg[(PIXEL_DATA_SIZE - ((i/384)+1)*384 + i%384)] = np.uint8(floats[i]*255)
        cvimg = np.reshape(cvimg, (288,-1))
        # display the image
        cv2.imshow('cvimg', cvimg)
        cv2.moveWindow('cvimg', 50, 50)
        waitkey = cv2.waitKey(1)
        if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
            print('exit key pressed')
            rospy.signal_shutdown('exit key pressed')
        elif waitkey == 99: # 'c' key
            print "C key pressed - recalibrating. Keep the lens Covered."
            calib_counter = 0

def temperature_process_first_sky(temperature_data):
    
    bwimg = np.zeros(PIXEL_DATA_SIZE, dtype=np.uint8)
    skymask = np.zeros(PIXEL_DATA_SIZE, dtype=bool)
    skyimg = np.zeros((PIXEL_DATA_SIZE), dtype=np.uint8)

    # turn ROS message into a numpy array
    tempData = np.asarray(temperature_data.data)
    
    # find the sky?
    for i in range(0, PIXEL_DATA_SIZE):
        if tempData[i] > -500 and tempData[i] < -300:
            skymask[i] = True

    # Temperature data into human viewable image
    temp_min = np.ndarray.min(tempData)
    temp_max = np.ndarray.max(tempData)
    temp_range = np.float(temp_max - temp_min)

    # Normalize temperatures so that the image is viewable by people
    floats = np.array((tempData - temp_min ) / (temp_range), dtype=float)
    bwimg = np.array(floats * 255, dtype = np.uint8)

    # now put the pixels into a opencv image compatible numpy array
    for i in range (0, PIXEL_DATA_SIZE):
        bwimg[(PIXEL_DATA_SIZE - ((i/384)+1)*384 + i%384)] = np.uint8(floats[i]*255)
        if skymask[i]:
            skyimg[(PIXEL_DATA_SIZE - ((i/384)+1)*384 + i%384)] = 255
    bwimg = np.reshape(bwimg, (288,-1))
    skyimg = np.reshape(skyimg, (288,-1))

    # Apply a heatmap to the thermal image for easire viewing
    heatmapped = cv2.applyColorMap(bwimg, cv2.COLORMAP_BONE)


    # display the image
    cv2.imshow('cvimg', heatmapped)
    cv2.moveWindow('cvimg', 50, 50)
    waitkey = cv2.waitKey(1)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')
    elif waitkey == 99: # 'c' key
        print "C key pressed - recalibrating. Keep the lens Covered."





def temp_sub():
    rospy.init_node('listener')
    rospy.Subscriber("temperature", Int16MultiArray, temperature_process_first_sky)
    rospy.spin()

def myshutdown():
    print('shutting down')
    cv2.destroyAllWindows

if __name__ == '__main__':
    temp_sub()



