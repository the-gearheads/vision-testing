#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>
#include "json.hpp"
using json = nlohmann::json;

extern "C" {
#include "apriltag.h"
#include "tag16h5.h"
}

/* Port and host added later */
#define UDPSINK_PIPELINE "rtph264pay config-interval=1 ! udpsink sync=false"

#define HWENC_PIPELINE "appsrc ! videoconvert ! v4l2h264enc extra-controls='controls,h264_profile=0,video_bitrate_mode=0,video_bitrate=3000000,h264_i_frame_period=1' ! 'video/x-h264, level=(string)5' ! h264parse ! " UDPSINK_PIPELINE
#define SOFTWARE_PIPELINE "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=1000 ! video/x-h264, level=(string)5 ! h264parse ! " UDPSINK_PIPELINE

#define FONT FONT_HERSHEY_PLAIN

using namespace cv;
int main(int argc, char** argv )
{
    std::string config_path = "config.json";
    if(argc > 1) {
        config_path = argv[1];
    }
    /* Open config.json and verify it exists*/
    std::ifstream f(config_path);
    json config;
    if(!f) {
        printf("Failed to open %s. Does it exist?\n", config_path.c_str());
        printf("Usage: %s [config file]\n", argv[0]);
        printf("Continuing anyways with defaults...\n");
        config = json::parse("{}");
    } else {
        config = json::parse(f, nullptr, true, true);
    }

    int cam_index = config.value("cam_id", 0);
    VideoCapture cap;
    if(config.value("cam_force_backend_v4l2", false)) {
        cap = VideoCapture(cam_index, CAP_V4L2);
    } else if (config.value("cam_force_backend_dshow", false)) {
        cap = VideoCapture(cam_index, CAP_DSHOW);
    } else if (config.value("cam_force_backend_gstreamer", false)) {
        cap = VideoCapture(cam_index, CAP_GSTREAMER);
    } else {
        cap = VideoCapture(cam_index, CAP_ANY);
    }

    if(!cap.isOpened()) {
        printf("Couldn't open camera %d\n", cam_index);
        exit(-1);
    }

    if(config.value("cam_width", 0))
        cap.set(CAP_PROP_FRAME_WIDTH, config.value("cam_width", 0));
    if(config.value("cam_height", 0))
        cap.set(CAP_PROP_FRAME_HEIGHT, config.value("cam_height", 0));
    if(config.value("cam_fps", 0))
        cap.set(CAP_PROP_FPS, config.value("cam_fps", 0));

    int width = cap.get(CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    int fps = cap.get(CAP_PROP_FPS);
    printf("%dx%d @ %dfps\n", width, height, fps);

    std::stringstream pipeline("");
    if(config.value("pihwenc", false)) {
        pipeline << HWENC_PIPELINE;
    } else {
        pipeline << SOFTWARE_PIPELINE;
    }

    pipeline << " host=" << config.value("udphost", "255.255.255.255");
    pipeline << " port=" << config.value("udpport", 5010);
    VideoWriter writer = VideoWriter(pipeline.str(), VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(width, height));

    /* Apriltag init */
    apriltag_family_t* tf = tag16h5_create();
    apriltag_detector_t* detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, tf);

    detector->quad_decimate = config.value("apriltag_quad_decimate", 2);
    detector->quad_sigma = config.value("apriltag_blur", 0);

    detector->nthreads = config.value("apriltag_threads", 0);
    if(!detector->nthreads) detector->nthreads = std::thread::hardware_concurrency();
    if(!detector->nthreads) detector->nthreads = 1;
    printf("Using %d threads\n", detector->nthreads);

    detector->debug = config.value("apriltag_debug", false);
    detector->refine_edges = config.value("apriltag_refine_edges", true);

    Mat img, greyImg;
    TickMeter meter;
    while(true) {
        cap >> img;
        meter.start();
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
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, FONT, fontscale, 2,
                                            &baseline);
            putText(img, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    FONT, fontscale, Scalar(0xff, 0x99, 0), 2);
        }

        /* FPS Meter Rendering*/
        int fontSize = 2;
        int baseline;
        std::string fps = std::to_string(meter.getFPS()) + " FPS";
        Size ts = getTextSize(fps, FONT, fontSize, 2, &baseline);
        /* Offset it 100px down */
        ts.height += 100;
        putText(img, fps, ts, FONT, fontSize, Scalar(255, 255, 255), 2);

        writer.write(img);
        apriltag_detections_destroy(detections);
        meter.stop();
    }

    apriltag_detector_destroy(detector);
    tag16h5_destroy(tf);

    return 0;
}