#pragma once
class ApriltagSettings {
    public:
    int quad_decimate;
    int blur;
    bool debug;
    bool refine_edges;
    int threads;
};

enum NTMode {
    SERVER, CLIENT, CLIENT_TEAM
};

class NTSettings {
    public:
    NTMode mode;
    int port;
    int team_number;
    std::string ip;
    std::string rootPrefix;
};

#define FOURCC(a,b,c,d) ( (uint32_t) (((d)<<24) | ((c)<<16) | ((b)<<8) | (a)) )

enum CameraBackend {
    ANY = CAP_ANY,
    GSTREAMER = CAP_GSTREAMER,
    V4L2 = CAP_V4L2,
    DSHOW = CAP_DSHOW
};

class CameraSettings {
    public:
    int width, height, fps;
    int force_fourcc;
    int id;
    bool use_hwenc;
    std::string ip;
    unsigned int port;
    CameraBackend backend;
};

class Config {
    public:
    static void init(json config);
    static NTSettings* nt;
    static ApriltagSettings* atag;
    static CameraSettings* cam;
    private:
    static void config_cam(json config);
    static void config_nt(json config);
    static void config_apriltag(json config);
};