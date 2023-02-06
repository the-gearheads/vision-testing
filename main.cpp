#include "ApriltagDetect.h"
#include "Config.h"

/* Port and host added later */
#define UDPSINK_PIPELINE "rtph264pay config-interval=1 ! udpsink sync=false"

#define HWENC_PIPELINE "appsrc ! videoconvert ! v4l2h264enc extra-controls=controls,h264_profile=0,video_bitrate_mode=0,video_bitrate=3000000,h264_i_frame_period=1 ! video/x-h264,level=(string)5 ! h264parse ! " UDPSINK_PIPELINE
#define SOFTWARE_PIPELINE "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=1000 ! video/x-h264, level=(string)5 ! h264parse ! " UDPSINK_PIPELINE

void start_nt(NT_Inst ntInst) {
   switch (Config::nt->mode) {
        case CLIENT:
            nt::StartClient(ntInst, Config::nt->ip.c_str(), Config::nt->port);
            break;
        case CLIENT_TEAM:
            nt::StartClientTeam(ntInst, Config::nt->team_number, Config::nt->port);
            break;
        case SERVER:
            nt::StartServer(ntInst, "./persistent.ini", Config::nt->ip.c_str(), Config::nt->port);
            break;
   }
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

    Config::init(config);

    VideoCapture cap = VideoCapture(Config::cam->id, Config::cam->backend);

    if(!cap.isOpened()) {
        printf("Couldn't open camera %d\n", Config::cam->id);
        exit(-1);
    }

    if(Config::cam->width)
        cap.set(CAP_PROP_FRAME_WIDTH, Config::cam->width);
    if(Config::cam->height)
        cap.set(CAP_PROP_FRAME_HEIGHT, Config::cam->height);
    if(Config::cam->fps)
        cap.set(CAP_PROP_FPS, Config::cam->fps);
    if(Config::cam->force_fourcc)
        cap.set(CAP_PROP_FOURCC, Config::cam->force_fourcc);

    int width = cap.get(CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    int fps = cap.get(CAP_PROP_FPS);
    printf("%dx%d @ %dfps\n", width, height, fps);

    std::stringstream pipeline("");
    if(Config::cam->use_hwenc) {
        pipeline << HWENC_PIPELINE;
    } else {
        pipeline << SOFTWARE_PIPELINE;
    }

    pipeline << " host=" << Config::cam->ip;
    pipeline << " port=" << Config::cam->port;
    VideoWriter writer = VideoWriter(pipeline.str(), VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(width, height));

    /* Initialize NetworkTables */
    NT_Inst ntInst = nt::GetDefaultInstance();
    start_nt(ntInst);
    nt::SetNetworkIdentity(ntInst, "vision");

    ApriltagDetect detector(config, ntInst);
    Mat img;
    TickMeter meter;
    double lastLoopTime = 0;
    while(true) {
        cap >> img;
        meter.start();
        waitKey(1);

        if(Config::atag->enabled) {
            detector.execute(img, lastLoopTime);
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
        meter.stop();
        lastLoopTime = meter.getAvgTimeMilli();
    }
    return 0;
}
