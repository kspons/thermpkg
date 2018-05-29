# include "ros/ros.h"
# include "std_msgs/Int16MultiArray.h"
# include "std_msgs/Int8.h"
# include <image_transport/image_transport.h>

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
    ros::Publisher pub_temperature2= n.advertise<std_msgs::Int16MultiArray>("temperature2", 1);

    // ros::Publisher pub_image = n.advertise<image_transport::ImageTransport>("thermal/image", 1);
    // image_transport::ImageTransport it(n);
    // image_transport::Publisher pub_img = it.advertise("thermal/image", 1);
    ros::Rate loop_rate(16);
    std_msgs::Int16MultiArray temp_msg;
    std_msgs::Int16MultiArray temp_msg2;


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


    // Calibration time
    uint8_t img[165888];
    
    double pre_offset_cal = 0;
    double gain_cal = 1;
    double offset_cal = 0;
    long meancal = 0;
    short frame1[PIXELS_DATA_SIZE];
    int d_frame1[PIXELS_DATA_SIZE];
    int my_frame[PIXELS_DATA_SIZE];
    int my_frame2[PIXELS_DATA_SIZE];

    int image_cal[PIXELS_DATA_SIZE];
    int deadpixel_map[PIXELS_DATA_SIZE] = { 0 };
    std::cout<<"Calibrating. Cover the lens."<<std::endl;

    // Identify dead Pixels
    while(!thermapp_GetImage(therm, frame1));
    while(!thermapp_GetImage(therm, frame1));


    for (int i = 0; i < PIXELS_DATA_SIZE; i++) {
        d_frame1[i] = frame1[i];
    }

    for (int i = 0; i < 20; i++) {
        std::cout<<"calibration frame "<<i+1<<" captured. keep lens covered"<<std::endl;
        while(!thermapp_GetImage(therm, frame1));
        for (int j = 0; j < PIXELS_DATA_SIZE; j++) {
            d_frame1[j] += frame1[j];
        }
    }

    for (int i = 0; i < PIXELS_DATA_SIZE; i++) {
        image_cal[i] = d_frame1[i] / 20;
        meancal+=image_cal[i];
    }

    meancal = meancal / PIXELS_DATA_SIZE;


    printf("meancal value is %d\n", meancal);
    printf("frame[0] value is %d\n", frame1[0]);


    // Log dead pixels
    for(int i = 0; i < PIXELS_DATA_SIZE; i++){
        if ((image_cal[i] > meancal + 250) || (image_cal[i] < meancal - 250)) {
		printf("Dead pixel ID: %d (%d vs %li)\n", i, image_cal[i], meancal);
		deadpixel_map[i] = 1;
	    }
    }

    // loop for publishing in ROS
    while (ros::ok())
    {
        if (thermapp_GetImage(therm, frame)) { //do this work when there's a new frame
//     // Replace dead pixels with the pixel value of its neighboor
//     while (1) {
//         if (thermapp_GetImage(therm, frame)) {
            int i;
            // int frameMax = ((frame[0] + pre_offset_cal - image_cal[0]) * gain_cal) + offset_cal;
            // int frameMin = ((frame[0] + pre_offset_cal - image_cal[0]) * gain_cal) + offset_cal;
            // for (i = 0; i < PIXELS_DATA_SIZE; i++) { // get the min and max values
            //     // only bother if the pixel isn't dead
            //     if (!deadpixel_map[i]) {
            //         int x = ((frame[i] + pre_offset_cal - image_cal[i]) * gain_cal) + offset_cal;
            //         if (x > frameMax) {
            //             frameMax = x;
            //         }
            //         if (x < frameMin) {
            //             frameMin = x;
            //         }
            //     }
            // }
            // second time through, this time actually scaling data
            for (i = 0; i < PIXELS_DATA_SIZE; i++) {
                int x;
                if (deadpixel_map[i]) {
                    x = ((frame[i-1] + pre_offset_cal - image_cal[i-1]) * gain_cal) + offset_cal;
                }else{
                    x = ((frame[i] + pre_offset_cal - image_cal[i]) * gain_cal) + offset_cal;
                }
                // x = (((double)x - frameMin)/(frameMax - frameMin))*255;
                // ^^^commented out because i don't want normalized data in the python script
                // my_frame[PIXELS_DATA_SIZE - 1 - (PIXELS_DATA_SIZE - ((i/384)+1)*384 + i%384)] = x;
                my_frame2[(PIXELS_DATA_SIZE - ((i/384)+1)*384 + i%384)] = x;                
                
            }

//             std::cout<<"frame max :"<<frameMax<<std::endl;
//             std::cout<<"frame min :"<<frameMax<<std::endl;
           
//             write(fdwr, img, 165888);
//         }
//     }
//     close(fdwr);
//     thermapp_Close(therm);
//     return 0;
// }



        // if (thermapp_GetImage(therm, frame)) { // do this work when there's a new frame
            temp_msg.data.clear();
            temp_msg2.data.clear();
            // loop for putting frame data into message format
            for (int i = 0; i < PIXELS_DATA_SIZE; i++)
            {
                // temp_msg.data[i] = frame[i];               
                // temp_msg.data.push_back(my_frame[i]);
                temp_msg2.data.push_back(my_frame2[i]);

            }
            // std::cout<<temp_msg.data[111]<<std::endl;
            pub_temperature.publish(temp_msg2);
            // pub_temperature2.publish(temp_msg2);
        }
        loop_rate.sleep();
    }

    close(fdwr);
    thermapp_Close(therm);

    return 0;
}