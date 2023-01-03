#include "Config.h"

/* Long story short, due to a weirdness in C++ static variables, they need to be declared separately.
 * The function declarations are below, but we still need to declare our other properties.  */
NTSettings* Config::nt;
ApriltagSettings* Config::atag;
CameraSettings* Config::cam;

void Config::init(json config) {
    nt = new NTSettings();
    atag = new ApriltagSettings();
    cam = new CameraSettings();
    
    config_nt(config);
    config_cam(config);
    config_apriltag(config);
}

void Config::config_cam(json config) {
    cam->width = config.value("cam_width", 640);
    cam->height = config.value("cam_height", 480);
    cam->fps = config.value("cam_fps", 30);
    cam->id = config.value("cam_id", 0);
    cam->use_hwenc = config.value("pihwenc", false);
    cam->ip = config.value("cam_broadcast_ip", "255.255.255.255");
    cam->port = config.value("cam_broadcast_port", 5010);
    cam->fx = config.value("cam_intrinsics_fx", 16);
    cam->fy = config.value("cam_intrinsics_fy", 9);
    cam->cx = config.value("cam_intrinsics_cx", 160);
    cam->cy = config.value("cam_intrinsics_cy", 90);

    std::string fourcc = config.value("force_fourcc", "");
    if (fourcc.length() == 4) {
        cam->force_fourcc = FOURCC(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
    } else {
        cam->force_fourcc = 0;
    }

    std::string backend = config.value("force_backend", "");
    if(backend.empty())
        backend = "any";
    else if(backend == "gstreamer")
        cam->backend = ANY;
    else if(backend == "dshow")
        cam->backend = GSTREAMER;
    else if(backend == "v4l2")
        cam->backend = V4L2;
    else if(backend == "any")
        cam->backend = ANY;
    else {
        printf("Unknown backend %s, using 'any'\n", backend.c_str());
        cam->backend = ANY;
    }
}

void Config::config_apriltag(json config) {
    atag->blur = config.value("apriltag_blur", 2.0);
    atag->quad_decimate = config.value("apriltag_quad_decimate", 4.0);
    atag->threads = config.value("apriltag_threads", 0);
    if(!atag->threads) { 
        atag->threads = std::thread::hardware_concurrency();
    }
    atag->refine_edges = config.value("apriltag_refine_edges", true);
    atag->debug = config.value("apriltag_debug", false);
    atag->areaThreshold = config.value("area_threshold", 0.0);
    atag->hammingThreshold = config.value("hamming_threshold", 1);
    atag->decisionMarginThreshold = config.value("decision_margin_threshold", 0.0);
}

void Config::config_nt(json config) {
    bool is_server = config.value("nt_server", false);
    nt->rootPrefix = config.value("nt_prefix", "/photonvision");
    nt->targetName = config.value("target_name", "target");
    nt->pre_beta_4_compat = config.value("pre_2023_1_1_beta_compat", false);
    nt->fullPath = nt->rootPrefix + "/" + nt->targetName;
    nt->reportPhotonVersion = config.value("report_photon_version", "v2023.1.1-beta-6");
    std::string teamnum_or_ip = config.value("nt_client_ip_or_teamnum", "127.0.0.1");
    /* If there isn't a . in the string, it's like a team number */
    bool is_team_num = (teamnum_or_ip.find(".") == std::string::npos);
    if (is_server) {
        nt->mode = SERVER;
        nt->ip = "0.0.0.0";
    } else {
        if(is_team_num) {
            nt->mode = CLIENT_TEAM;
            nt->ip = "";
            try {
                nt->team_number = std::stoi(teamnum_or_ip);
            } catch (...) {
                printf("Failed to convert team number %s to int", teamnum_or_ip.c_str());
                nt->team_number = 0;
            }
        } else {
            nt->mode = CLIENT;
            nt->ip = teamnum_or_ip;
        }
    }

    nt->port = config.value("nt_port", 1735);
    if(!nt->port) {
        nt->port = 1735;
    }
}