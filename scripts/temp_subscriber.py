#! /usr/bin/env python2

import sys
import rospy
import cv2
import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int16MultiArray

import os

PIXEL_DATA_SIZE = int(384*288)
# stupid globalvars
counter = 0

# Calibration variables
calib_counter = 0
pix_calib = np.zeros(PIXEL_DATA_SIZE)
# deadpixel_map = np.zeros(PIXEL_DATA_SIZE)
deadpixel_map = [0]

def nothing(x): # pass
    pass

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
    
    cv2.namedWindow('image')
    cv2.namedWindow('image2')
    cv2.createTrackbar('Min', 'image', 0, 500, nothing)
    cv2.createTrackbar('Max', 'image', 0, 500, nothing)
    track_min = cv2.getTrackbarPos('Min', 'image') - 325
    track_max = cv2.getTrackbarPos('Max', 'image') - 250

    print 'min : ', track_min
    print 'max : ', track_max, '\n'

    bwimg = np.zeros(PIXEL_DATA_SIZE, dtype=np.uint8)
    skymask = np.zeros(PIXEL_DATA_SIZE, dtype=bool)
    skyimg = np.zeros((PIXEL_DATA_SIZE), dtype=np.uint8)

    # turn ROS message into a numpy array
    tempData = np.asarray(temperature_data.data)

    # Temperature data into human viewable image
    temp_min = np.ndarray.min(tempData)
    temp_max = np.ndarray.max(tempData)
    temp_range = np.float(temp_max - temp_min)

    print 'temp_min : ', temp_min
    print 'temp_max : ', temp_max, '\n'

     # find the sky?
    for i in range(0, PIXEL_DATA_SIZE):
        if tempData[i] > track_min and tempData[i] < track_max:
            skymask[i] = True

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
    heatmapped = cv2.flip(cv2.applyColorMap(bwimg, cv2.COLORMAP_BONE), 1)
    heatmapped[:,:,0] = cv2.flip(skyimg, 1)


    # display the image
    cv2.imshow('image', heatmapped)
    cv2.imshow('image2', skyimg)
    waitkey = cv2.waitKey(1)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')

def minimal(temperature_msg): # displays normalized img

    # put the temperature message into a a 2x2 numpy array
    tempData = np.zeros((288,384), dtype=int)
    for i in range(0, 288):
        for j in range(0, 384):
            tempData[i, j] = temperature_msg.data[PIXEL_DATA_SIZE - 1 - (i*384+j)]

    # normalize the pixels between 0 and 255 to make it viewable
    temp_min = min(temperature_msg.data)
    temp_max = max(temperature_msg.data)
    temp_range = float(temp_max - temp_min)
    bwimg = (tempData - temp_min) / temp_range * 255
    bwimg = bwimg.astype('uint8')
    
    # display the image
    cv2.imshow('image', bwimg)
    waitkey = cv2.waitKey(1)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')

def simple_img(tempData): # outputs a viewable image
    # input is 2D numpy array of temperature data
    # normalize the pixels between 0 and 255 to make it viewable
    temp_min = tempData.min()
    temp_max = tempData.max()
    temp_range = float(temp_max - temp_min)
    bwimg = (tempData - temp_min) / temp_range * 255
    bwimg = bwimg.astype('uint8')
    return bwimg

def newheatmap(temperature_msg):
    # put the temperature message into a a 2x2 numpy array
    tempData = np.zeros((288,384), dtype=int)
    for i in range(0, 288):
        for j in range(0, 384):
            tempData[i, j] = temperature_msg.data[PIXEL_DATA_SIZE - 1 - (i*384+j)]

    # normalize the pixels between 0 and 255 to make it viewable
    temp_min = min(temperature_msg.data)
    temp_max = max(temperature_msg.data)
    temp_range = float(temp_max - temp_min)
    bwimg = (tempData - temp_min) / temp_range * 255
    bwimg = bwimg.astype('uint8')

    # initialize a color image
    myimg = cv2.cvtColor(bwimg, cv2.COLOR_GRAY2BGR)
    # select ranges and color them
    for i in range(0,288):
        for j in range(0, 384):
            if tempData[i,j] >= -700 and tempData[i,j] < -430: # around -330 for Memorial .bags
                myimg[i,j] = [255,0,0]
    # display the image
    cv2.imshow('image', myimg)
    waitkey = cv2.waitKey(1)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')

def gracefulbug(temperature_msg):
    # put the temperature message into a a 2x2 numpy array
    initguess = temperature_msg.data[0] + 100
    temp_min = initguess
    temp_max = max(temperature_msg.data)
    temp_range = float(temp_max - temp_min)
    myimg = np.zeros((288,384,3), dtype='uint8')
    for i in range(0, 288):
        for j in range(0, 384):
            x = temperature_msg.data[PIXEL_DATA_SIZE - 1 - (i*384+j)]
            if temperature_msg.data < initguess:
                myimg[i,j,] = [255,0,0]
            else:
                pix = (x - temp_min) / temp_range * 255
                myimg[i,j] = np.array((pix,pix,pix), dtype = 'uint8')

    # display the image
    cv2.imshow('image', myimg)
    waitkey = cv2.waitKey(1)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')

def newheatmap2(temperature_msg): #displays blue 'sky' and grey other
    # put the temperature message into a a 2x2 numpy array
    initguess = temperature_msg.data[PIXEL_DATA_SIZE - 1] + 100
    temp_min = initguess
    temp_max = max(temperature_msg.data)
    temp_range = float(temp_max - temp_min)
    myimg = np.zeros((288,384,3), dtype='uint8')
    for i in range(0, 288):
        for j in range(0, 384):
            x = temperature_msg.data[PIXEL_DATA_SIZE - 1 - (i*384+j)]
            if x < initguess:
                myimg[i,j,] = [255,0,0]
            else:
                pix = (x - temp_min) / temp_range * 255
                myimg[i,j] = np.array((pix,pix,pix), dtype = 'uint8')

    # display the image
    cv2.imshow('image', myimg)
    waitkey = cv2.waitKey(1)
    imagesaver(myimg)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')

def newheatmap3(temperature_msg): #displays blue 'sky', red 'sun', and grey other
    # put the temperature message into a a 2x2 numpy array
    initguess = temperature_msg.data[PIXEL_DATA_SIZE - 1] + 50
    temp_min = initguess
    temp_max = max(temperature_msg.data)
    temp_range = float(temp_max - temp_min)
    myimg = np.zeros((288,384,3), dtype='uint8')
    for i in range(0, 288):
        for j in range(0, 384):
            x = temperature_msg.data[PIXEL_DATA_SIZE - 1 - (i*384+j)]
            if x < initguess:
                myimg[i,j,] = [255,0,0]
            elif x > 0:
                myimg[i,j,] = [0,0,255]
            else:
                pix = (x - temp_min) / temp_range * 255
                myimg[i,j] = np.array((pix,pix,pix), dtype = 'uint8')
    print temp_max
    # display the image
    cv2.imshow('image', myimg)
    waitkey = cv2.waitKey(1)
    imagesaver(myimg)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')

def Temperature_as_np_array(temp_msg): # put ROS message of temperature into np 2D array
    tempar = np.zeros((288,384), dtype = int)
    for i in range(0, 288):
        for j in range(0, 384):
            tempar[i,j] = temp_msg.data[PIXEL_DATA_SIZE - 1 - (i*384 + j)]
    return tempar

def histogram(tempData): # creates a histogram of temperature image (tempData is 288x384 np array)
    # hist = cv2.calcHist(tempData, [0], None, [256], [-500,0])
    bin_num = 100
    hist, bin_edges = np.histogram(tempData, bin_num, range=None, normed=False, weights=None)


    # print hist
    hist_img = np.zeros((256*3,256*2))
    hist_max = float(hist.max())
    for i in range (0, bin_num):
        spacing = np.size(hist_img, 1) / bin_num
        line_height = int(hist[i]/hist_max*256)
        cv2.line(hist_img, (i*spacing,50), (i*spacing,line_height+50), (255,0,0), 1)
    hist_img = cv2.flip(hist_img, 0)
    cv2.putText(hist_img, str(bin_edges[0]), (0,256*3), cv2.FONT_HERSHEY_COMPLEX, 1, (256,0,0))
    cv2.putText(hist_img, str(bin_edges[-1]), (256*2-100,256*3), cv2.FONT_HERSHEY_COMPLEX, 1, (256,0,0))

    return hist_img
    
def horiz_hist(tempData): # returns an image of the 'horizontal' histogram
    # this function takes the average of each row and plots it

    # get the average
    row_avg = np.average(tempData, axis=1)

    # initialize opencv image as np array
    hist_img = np.zeros((256+200,256*2), dtype='uint8')

    # get values that will be used to scale data to fit it onto the image
    mymax = row_avg.max()
    mymin = row_avg.min()
    myrange = abs(mymax - mymin)

    # loop that plots a circle for each average row value
    for i in range(np.size(tempData, axis=0)):
        # mypt = (row of image, average value of said row)
        mypt = (100+i, int((row_avg[i]-mymin)/myrange*256)+100)
        cv2.circle(hist_img, mypt, 1, (255,0,0), 1)

    hist_img = cv2.flip(hist_img, 0)
    cv2.putText(hist_img, str(int(mymin)), (50,356), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0))
    cv2.putText(hist_img, str(int(mymax)), (50,100), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0))

    return hist_img

def img_merge(img0, img1):
    height = np.zeros((2), dtype=int)
    width = np.zeros((2), dtype=int)
    [height[0], width[0]] = np.shape(img0)
    [height[1], width[1]] = np.shape(img1)

    final_img = np.zeros((height.max(), width.sum()), dtype='uint8')
    startpos = (height.max()-height.min())/2
    final_img[0:height[1], width[0]:] = img1
    final_img[startpos:startpos+height[0], 20:width[0]+20] = img0



    final_img = cv2.cvtColor(final_img, cv2.COLOR_GRAY2BGR)

    cv2.arrowedLine(final_img, (20, startpos), (20, startpos+height.min()), (255,0,0), 5)
    cv2.arrowedLine(final_img, (width[0]+100, height.max()-50), (width[0]+100+288, height.max()-50), (255,0,0), 5)

    # green reference lines
    # LHS
    cv2.line(final_img, (20,height.max()/2), (20+384, height.max()/2), (0,255,0), 2)
    # RHS
    cv2.line(final_img, (width[0]+100+144,100), (width[0]+100+144, 100+256), (0,255,0), 2)


    return final_img

def sky_picture(tempData): # create and display image of detected sky 
    # input tempData is a 2D numpy array, dtype int, of temperature data
    initguess =tempData[0,0] - 50
    temp_min = float(initguess)
    temp_max = tempData.max()
    temp_range = float(temp_max - temp_min)
    myimg = np.zeros((288,384,3), dtype='uint8')
    for i in range(0, 288):
        for j in range(0, 384):
            if tempData[i,j] < initguess:
                myimg[i,j,] = [255,0,0]
            elif tempData[i,j] > 0:
                myimg[i,j,] = [0,0,255]
            else:
                pix = (tempData[i,j] - temp_min)  * 255
                myimg[i,j] = np.array((pix,pix,pix), dtype = 'uint8')

    return myimg

def imagesaver(img):
    global counter
    # change cwd
    foldername = "ceiling_wall"
    folderpath = "/home/kspons/Pictures/" + foldername
    imgname = 'img' + str(counter).zfill(4) + ".png"
    if not os.path.exists(folderpath):
        os.makedirs(folderpath)
        print "creating folder " + folderpath
    cv2.imwrite(folderpath + "/" + imgname, img)
    counter = counter + 1

def fuckeverything(msg):
    temp = np.zeros((288,384), dtype=int)
    for i in range(0, 288):
        for j in range(0, 384):
            tempData[i, j] = temperature_msg.data[PIXEL_DATA_SIZE - 1 - (i*384+j)]

    # put the temperature message into a a 2x2 numpy array
    tempData = np.zeros((288,384), dtype=int)
    for i in range(0, 288):
        for j in range(0, 384):
            tempData[i, j] = temperature_msg.data[PIXEL_DATA_SIZE - 1 - (i*384+j)]

    # normalize the pixels between 0 and 255 to make it viewable
    temp_min = min(temperature_msg.data)
    temp_max = max(temperature_msg.data)
    temp_range = float(temp_max - temp_min)
    bwimg = (tempData - temp_min) / temp_range * 255
    bwimg = bwimg.astype('uint8')
    
    # display the image
    cv2.imshow('image', bwimg)
    waitkey = cv2.waitKey(1)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')

def temperature_callback(temp_msg):
    tempData = Temperature_as_np_array(temp_msg)

    # # Select which images you want to create
    # min_img = simple_img(tempData)
    # hist_img = histogram(tempData)
    hhist_img = horiz_hist(tempData)
    # merge_img = img_merge(min_img, hhist_img)
    # sky_img = sky_picture(tempData)

    # # Save an image if you want to
    # imagesaver(img)

    # # Display selected images
    # cv2.imshow('min_img', min_img)
    # cv2.imshow('histogram', hist_img)
    cv2.imshow('hhist_img', hhist_img)
    # cv2.imshow('merge_image', merge_img)
    # cv2.imshow('sky_img', sky_img)
    
    waitkey = cv2.waitKey(1)
    if waitkey == 27 or waitkey == 113: # 'esc' and 'q' keys
        print('exit key pressed')
        rospy.signal_shutdown('exit key pressed')


def temp_sub():
    rospy.init_node('listener')
    rospy.Subscriber("temperature", Int16MultiArray, temperature_callback)
    rospy.spin()

def myshutdown():
    print('shutting down, see ya')
    cv2.destroyAllWindows

if __name__ == '__main__':
    temp_sub()



