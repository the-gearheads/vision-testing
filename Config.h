#pragma once
class ApriltagSettings {
    public:
    bool enabled;
    float quad_decimate;
    float blur;
    bool debug;
    bool refine_edges;
    int threads;
    int hammingThreshold;
    double areaThreshold;
    float decisionMarginThreshold;
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
    std::string targetName;
    std::string fullPath;
    std::string reportPhotonVersion;
    bool pre_beta_4_compat;
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
    bool pi_hwenc;
    bool va_hwenc;
    bool show_fps;
    std::string ip;
    unsigned int port;
    CameraBackend backend;
    double fx;
    double fy;
    double cx;
    double cy;
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
