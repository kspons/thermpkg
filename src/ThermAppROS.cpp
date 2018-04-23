# include "ros/ros.h"
# include "std_msgs/Int16MultiArray.h"
# include "std_msgs/Int8.h"

#include <sstream>

#include "thermapp.h"

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>


#define PIXELS_DATA_SIZE	(384*288)
#define VIDEO_DEVICE "/dev/video0"
#define FRAME_WIDTH  384
#define FRAME_HEIGHT 288
//#define FRAME_FORMAT V4L2_PIX_FMT_GREY
//#define FRAME_FORMAT V4L2_PIX_FMT_YVU420
#define FRAME_FORMAT V4L2_PIX_FMT_YVU420
#define ROUND_UP_2(num) (((num)+1)&~1)
#define ROUND_UP_4(num) (((num)+3)&~3)
#define ROUND_UP_8(num)  (((num)+7)&~7)
#define ROUND_UP_16(num) (((num)+15)&~15)
#define ROUND_UP_32(num) (((num)+31)&~31)
#define ROUND_UP_64(num) (((num)+63)&~63)

int format_properties(const unsigned int format,
                      const unsigned int width,
                      const unsigned int height,
                      size_t*linewidth,
                      size_t*framewidth) {
    unsigned int lw, fw;
    switch(format) {
    case V4L2_PIX_FMT_YUV420:
        lw = width; /* ??? */
        fw = ROUND_UP_4 (width) * ROUND_UP_2 (height);
        fw += 2 * ((ROUND_UP_8 (width) / 2) * (ROUND_UP_2 (height) / 2));
        break;
    case V4L2_PIX_FMT_YVU420:
        lw = width; /* ??? */
        fw = ROUND_UP_4 (width) * ROUND_UP_2 (height);
        fw += 2 * ((ROUND_UP_8 (width) / 2) * (ROUND_UP_2 (height) / 2));
        break;
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_Y41P:
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_YVYU:
        lw = (ROUND_UP_2 (width) * 2);
        fw = lw * height;
        break;
    case V4L2_PIX_FMT_Y10:
        fprintf(stdout,"S/W\n");
        lw = width;
        fw = width * height;
        break;
    default:
        return 0;
    }
    fprintf(stdout,"framewidth %d\n", fw);
    fprintf(stdout,"linewidth %d\n", lw);
    if(linewidth)*linewidth=lw;
    if(framewidth)*framewidth=fw;

    return 1;
}


extern "C" int camerastart(int argc, char *argv[], short *frame);

int main(int argc, char** argv)
{
    // set up ros stuff
    ros::init(argc, argv, "thermal_node");
    ros::NodeHandle n;
    ros::Publisher pub_temperature = n.advertise<std_msgs::Int16MultiArray>("temperature", 1);
    ros::Rate loop_rate(10);
    std_msgs::Int16MultiArray temp_msg;


    // initialize camera (be sure to run sudo modprobe v4l2loopback before this)
    short frame[PIXELS_DATA_SIZE];

    ThermApp *therm = thermapp_initUSB();
    if (therm == NULL) 
    {
        std::cout<<"init error\n"<<std::endl;
        return -1;
    }
    if (thermapp_USB_checkForDevice(therm, VENDOR, PRODUCT) == -1){
       fputs("USB_checkForDevice error\n", stderr);
       return -1;
    } else {
        puts("thermapp_FrameRequest_thread\n");
        //Run thread usb therm
        thermapp_FrameRequest_thread(therm);
    }
    struct v4l2_capability vid_caps;
    struct v4l2_format vid_format;

    const char *video_device = VIDEO_DEVICE;
    int fdwr = open(video_device, O_RDWR);
    assert(fdwr >= 0);

    std::cout<<"ThermApp camera initialized"<<std::endl;

    int ret_code = ioctl(fdwr, VIDIOC_QUERYCAP, &vid_caps);
    assert(ret_code != -1);

    memset(&vid_format, 0, sizeof(vid_format));

    ret_code = ioctl(fdwr, VIDIOC_G_FMT, &vid_format);

    size_t framesize;
    size_t linewidth;
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    vid_format.fmt.pix.width = FRAME_WIDTH;
    vid_format.fmt.pix.height = FRAME_HEIGHT;
    vid_format.fmt.pix.pixelformat = FRAME_FORMAT;
    vid_format.fmt.pix.sizeimage = framesize;
    vid_format.fmt.pix.field = V4L2_FIELD_NONE;
    vid_format.fmt.pix.bytesperline = linewidth;
    vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

    ret_code = ioctl(fdwr, VIDIOC_S_FMT, &vid_format);
    
    std::cout<<"made it2"<<std::endl;

    if(!format_properties(vid_format.fmt.pix.pixelformat,
                          vid_format.fmt.pix.width, vid_format.fmt.pix.height,
                          &linewidth,
                          &framesize)) {
        printf("unable to guess correct settings for format '%d'\n", FRAME_FORMAT);
    }

    // loop for publishing in ROS
    while (ros::ok())
    {
        if (thermapp_GetImage(therm, frame)) { // do this work when there's a new frame
            temp_msg.data.clear();
            // loop for putting frame data into message format
            for (int i = 0; i < PIXELS_DATA_SIZE; i++)
            {
                // temp_msg.data[i] = frame[i];               
                temp_msg.data.push_back(frame[i]);
            }
            // std::cout<<temp_msg.data[111]<<std::endl;
            pub_temperature.publish(temp_msg);
        }
        // loop_rate.sleep();
    }

    close(fdwr);
    thermapp_Close(therm);

    return 0;
}


// this motherfucker works now