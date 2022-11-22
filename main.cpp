#include "ApriltagDetect.h"

/* Port and host added later */
#define UDPSINK_PIPELINE "rtph264pay config-interval=1 ! udpsink sync=false"

#define HWENC_PIPELINE "appsrc ! videoconvert ! v4l2h264enc extra-controls='controls,h264_profile=0,video_bitrate_mode=0,video_bitrate=3000000,h264_i_frame_period=1' ! 'video/x-h264, level=(string)5' ! h264parse ! " UDPSINK_PIPELINE
#define SOFTWARE_PIPELINE "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=1000 ! video/x-h264, level=(string)5 ! h264parse ! " UDPSINK_PIPELINE

void start_nt(json config, NT_Inst ntInst) {
    int nt_port = config.value("networktables_port", 1735);
    if(!nt_port) nt_port = 1735;

    if(!config.value("networktables_ip", "").empty()) {
        nt::StartClient(ntInst, config.value("networktables_ip", "").c_str(), nt_port);
        return;
    }

    if(config.value("team_number", 0)) {
        nt::StartClientTeam(ntInst, config.value("team_number", 0), nt_port);
        return;
    }

    nt::StartServer(ntInst, "./persistent.ini", "0.0.0.0", nt_port);
}

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

    /* Initialize NetworkTables */
    NT_Inst ntInst = nt::GetDefaultInstance();
    start_nt(config, ntInst);
    nt::SetNetworkIdentity(ntInst, "vision");

    ApriltagDetect detector(config, ntInst);
    Mat img;
    TickMeter meter;
    while(true) {
        cap >> img;
        meter.start();
        waitKey(1);

        detector.execute(img);

        /* FPS Meter Rendering*/
        int fontSize = 2;
        int baseline;
        std::string fps = std::to_string(meter.getFPS()) + " FPS";
        Size ts = getTextSize(fps, FONT, fontSize, 2, &baseline);
        /* Offset it 100px down */
        ts.height += 100;
        putText(img, fps, ts, FONT, fontSize, Scalar(255, 255, 255), 2);

        writer.write(img);
        meter.stop();
    }
    return 0;
}
