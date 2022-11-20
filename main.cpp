#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include "json.hpp"

extern "C" {
#include "apriltag.h"
#include "tag16h5.h"
}

#define UDPSINK_PIPELINE "rtph264pay config-interval=1 ! udpsink host=192.168.0.255 port=5010 sync=false"

#define HWENC_PIPELINE "appsrc ! videoconvert ! v4l2h264enc extra-controls='controls,h264_profile=0,video_bitrate_mode=0,video_bitrate=3000000,h264_i_frame_period=1' ! 'video/x-h264, level=(string)5' ! h264parse ! " UDPSINK_PIPELINE
#define SOFTWARE_PIPELINE "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=1000 ! video/x-h264, level=(string)5 ! h264parse ! " UDPSINK_PIPELINE

using namespace cv;
int main(int argc, char** argv )
{
    int cam_index = 0;
    if(argc > 1) {
        cam_index = strtol(argv[1], nullptr, 10);
    }
    
    VideoCapture cap = VideoCapture(cam_index, CAP_V4L2);

    if(!cap.isOpened()) {
        printf("Couldn't open camera %d\n", cam_index);
        exit(-1);
    }

    int width = cap.get(CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    int fps = cap.get(CAP_PROP_FPS);
    printf("%dx%d @ %dfps\n", width, height, fps);

    VideoWriter writer = VideoWriter(SOFTWARE_PIPELINE, VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(width, height));

    /* Apriltag init */
    apriltag_family_t* tf = tag16h5_create();
    apriltag_detector_t* detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, tf);

    detector->quad_decimate = 2;
    detector->quad_sigma = 0;
    detector->nthreads = std::thread::hardware_concurrency();
    if(!detector->nthreads) detector->nthreads = 1;
    printf("Using %d threads\n", detector->nthreads);
    detector->debug = 0;
    detector->refine_edges = 1;

    Mat img, greyImg;
    TickMeter meter;
    meter.start();
    while(true) {
        cap >> img;
        cvtColor(img, greyImg, COLOR_BGR2GRAY);
        waitKey(1);

        // Make an image_u8_t header for the Mat data
        image_u8_t frame = { .width = greyImg.cols,
            .height = greyImg.rows,
            .stride = greyImg.cols,
            .buf = greyImg.data
        };

        zarray_t *detections = apriltag_detector_detect(detector, &frame);

        /* Do stuff with detections here */
                // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(img, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(img, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(img, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(img, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            std::stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(img, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }

        writer.write(img);
        apriltag_detections_destroy(detections);

    }

    apriltag_detector_destroy(detector);
    tag16h5_destroy(tf);

    return 0;
}