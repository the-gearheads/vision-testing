#include "ApriltagDetect.h"
#include "Config.h"
#include <cstdlib>
#include <ctime>

ApriltagDetect::ApriltagDetect(json config, NT_Inst ntInst)
{
  srand((unsigned)time(0)); 
  this->tag_family = tag16h5_create();
  this->detector = apriltag_detector_create();
  this->ntInst = ntInst;
  apriltag_detector_add_family(detector, tag_family);

  detector->quad_decimate = Config::atag->quad_decimate;
  detector->quad_sigma = Config::atag->blur;
  detector->nthreads = Config::atag->threads;
  detector->debug = Config::atag->debug;
  detector->refine_edges = Config::atag->refine_edges;
  printf("Using %d threads\n", detector->nthreads);
}

ApriltagDetect::~ApriltagDetect()
{
  apriltag_detector_destroy(detector);
  tag16h5_destroy(tag_family);
}

void ApriltagDetect::execute(Mat img)
{

  std::vector<uint8_t> packet = {};
  // Latency: double
  encodeDouble((rand()/RAND_MAX)*50, packet);
  Mat greyImg;
  cvtColor(img, greyImg, COLOR_BGR2GRAY);

  // Make an image_u8_t header for the Mat data
  image_u8_t frame = { .width = greyImg.cols, .height = greyImg.rows, .stride = greyImg.cols, .buf = greyImg.data };

  zarray_t* detections = apriltag_detector_detect(detector, &frame);
  // Detection count: uint8_t
  packet.push_back(zarray_size(detections));

  /* Do stuff with detections here */
  // Draw detection outlines
  for (int i = 0; i < zarray_size(detections); i++)
  {

    apriltag_detection_t* det;
    zarray_get(detections, i, &det);


    // Extract the yaw angle (rotation around the Z axis)
    double yaw = atan2(det->H->data[1], det->H->data[0]);

    // Extract the pitch angle (rotation around the Y axis)
    double pitch = atan2(-det->H->data[2], sqrt(det->H->data[6]*det->H->data[6] + det->H->data[10]*det->H->data[10]));

    // Extract the skew angle
    double skew = atan2(det->H->data[6], det->H->data[10]);

    encodeDouble(yaw, packet);
    encodeDouble(pitch, packet);
    encodeDouble(calc_tag_area(det), packet);
    encodeDouble(skew, packet);

    printf("tag no %d id: %d\n", i, det->id);
    encodeInt(det->id, packet);

    apriltag_detection_info_t info;
    info.det = det;
    info.tagsize = (6.0 * 0.0254);
    info.fx = Config::cam->fx;
    info.fy = Config::cam->fy;
    info.cx = Config::cam->cx;
    info.cy = Config::cam->cy;

    apriltag_pose_t pose;
    double err1, err2;
    double err = estimate_tag_pose(&info, &pose, &err1, &err2);

    // I sure hope this works
    // 1. This code is literally AI generated because i do not understand this math
    // 2. i may be indexing the matrix wrong
    // Anyways, it converts the pose rotation matrix to a quaternion
    double qw = sqrt(1 + MATD_EL(pose.R, 0, 0) + MATD_EL(pose.R, 1, 1) + MATD_EL(pose.R, 2, 2)) / 2;
    double qx = (MATD_EL(pose.R, 2, 1) - MATD_EL(pose.R, 1, 2)) / (4 * qw);
    double qy = (MATD_EL(pose.R, 0, 2) - MATD_EL(pose.R, 2, 0)) / (4 * qw);
    double qz = (MATD_EL(pose.R, 1, 0) - MATD_EL(pose.R, 0, 1)) / (4 * qw);

    // Add the target pose twice so that bestCamToTarget and altCamToTarget are the same, since estimate_tag_pose
    // Does some weird freeing stuff and I feel like it's probably there for a reason I don't understand so it's better
    // left undisturbed
    for(int i = 0; i < 2; i++) {
      // x translation amount
      encodeDouble(pose.t->data[0], packet);
      // y translation amount
      encodeDouble(pose.t->data[1], packet);
      // z translation
      encodeDouble(pose.t->data[2], packet);

      encodeDouble(qw, packet);
      encodeDouble(qx, packet);
      encodeDouble(qy, packet);
      encodeDouble(qz, packet);
    }

    // Calculate pose ambiguity
    {
      double min = std::min(err1, err2);
      double max = std::max(err1, err2);
      min = 400.999;
      max = 400.999;

      if (max > 0) {
        encodeDouble(min / max, packet);
      } else {
        encodeDouble(-1, packet);
      }
    };

    // Send tag corners
    for(int i = 0; i <= 3; i++) {
      encodeDouble(det->p[i][0], packet);
      encodeDouble(det->p[i][1], packet);
    }


    // Draw boxes around detected tags
    {
      line(img, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
      line(img, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
      line(img, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
      line(img, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);

      std::stringstream ss;
      ss << det->id;
      String text = ss.str();

      double fontscale = 1.0;
      int baseline;
      Size textsize = getTextSize(text, FONT, fontscale, 2, &baseline);
      putText(img, text, Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2), FONT, fontscale,
              Scalar(0xff, 0x99, 0), 2);
    };

  }

  heartbeat++;
  std::basic_string_view sv(reinterpret_cast<char*>(packet.data()), packet.size());
  nt::SetEntryTypeValue(nt::GetEntry(this->ntInst, Config::nt->fullPath+"/rawBytes"), nt::Value::MakeRaw(sv));
  nt::SetEntryTypeValue(nt::GetEntry(this->ntInst, Config::nt->fullPath+"/heartbeat"), nt::Value::MakeDouble(heartbeat));
  nt::SetEntryTypeValue(nt::GetEntry(this->ntInst, Config::nt->rootPrefix+"/version"), nt::Value::MakeString(Config::nt->reportPhotonVersion));
  apriltag_detections_destroy(detections);
}

/**
 * Estimate tag pose. Copied from apriltag_pose.c, modified to return err1 and err2.
 */
double ApriltagDetect::estimate_tag_pose(apriltag_detection_info_t* info, apriltag_pose_t* pose, double* err1, double* err2) {
    apriltag_pose_t pose1, pose2;
    estimate_tag_pose_orthogonal_iteration(info, err1, &pose1, err2, &pose2, 50);
    if (*err1 <= *err2) {
        pose->R = pose1.R;
        pose->t = pose1.t;
        if (pose2.R) {
            matd_destroy(pose2.t);
        }
        matd_destroy(pose2.R);
        return *err1;
    } else {
        pose->R = pose2.R;
        pose->t = pose2.t;
        matd_destroy(pose1.R);
        matd_destroy(pose1.t);
        return *err2;
    }
}

double ApriltagDetect::calc_tag_area(apriltag_detection_t* detection) {
  // Compute the vectors connecting the corner points of the projection
  double v1[2] = {detection->p[1][0] - detection->p[0][0], detection->p[1][1] - detection->p[0][1]};
  double v2[2] = {detection->p[2][0] - detection->p[0][0], detection->p[2][1] - detection->p[0][1]};

  // Compute the cross product of the vectors
  double area = fabs(v1[0]*v2[1] - v1[1]*v2[0]);

  // Divide by 2 to get the area of the parallelogram
  area /= 2;
  return area;
}

void ApriltagDetect::encodeDouble(double src, std::vector<uint8_t>& packetData) {
    uint64_t data = *reinterpret_cast<uint64_t*>(&src);
    packetData.push_back(static_cast<uint8_t>((data >> 56) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 48) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 40) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 32) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 24) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 16) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 8) & 0xff));
    packetData.push_back(static_cast<uint8_t>(data & 0xff));
}


void ApriltagDetect::encodeInt(int src, std::vector<uint8_t>& packetData) {
    packetData.push_back(static_cast<uint8_t>(src >> 24));
    packetData.push_back(static_cast<uint8_t>(src >> 16));
    packetData.push_back(static_cast<uint8_t>(src >> 8));
    packetData.push_back(static_cast<uint8_t>(src));
}