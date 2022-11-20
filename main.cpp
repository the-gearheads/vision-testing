#include <stdio.h>
#include <opencv2/opencv.hpp>

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
    int width = cap.get(CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    int fps = cap.get(CAP_PROP_FPS);
    printf("%dx%d @ %dfps\n", width, height, fps);

    VideoWriter writer = VideoWriter(SOFTWARE_PIPELINE, VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(width, height));

    while(true) {
        Mat img;
        if(cap.read(img)) {
            writer.write(img);
            imshow("Live", img);
            waitKey(1);
        }
    }

    return 0;
}