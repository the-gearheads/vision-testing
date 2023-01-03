#include "ApriltagDetect.h"
#include "PhotonCompat.h"
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

void ApriltagDetect::execute(Mat img, double lastLatencyVal)
{
  PhotonCompat::PhotonPacket packet;
  packet.latency = lastLatencyVal;

  Mat greyImg;
  cvtColor(img, greyImg, COLOR_BGR2GRAY);

  // Make an image_u8_t header for the Mat data
  image_u8_t frame = { .width = greyImg.cols, .height = greyImg.rows, .stride = greyImg.cols, .buf = greyImg.data };

  zarray_t* detections = apriltag_detector_detect(detector, &frame);  
  packet.detections.reserve(zarray_size(detections));

  for (int i = 0; i < zarray_size(detections); i++)
  {
    PhotonCompat::PhotonDetection detection;
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);

    if(det->hamming > Config::atag->hammingThreshold) { continue; }
    if(det->decision_margin < Config::atag->decisionMarginThreshold) { continue; }

    detection.tagId = det->id;

    detection.yaw = atan2(det->H->data[1], det->H->data[0]);
    detection.pitch = atan2(-det->H->data[2], sqrt(det->H->data[6]*det->H->data[6] + det->H->data[10]*det->H->data[10]));
    detection.skew = atan2(det->H->data[6], det->H->data[10]);
    detection.area = calc_tag_area(det);

    if(detection.area < Config::atag->areaThreshold) { continue; }

    printf("tag no %d id: %d dm: %f ham: %d area: %f\n", i, det->id, det->decision_margin, det->hamming, detection.area);

    apriltag_detection_info_t info;
    info.det = det;
    /* Tags are 6 inches */
    info.tagsize = (6.0 * 0.0254);
    info.fx = Config::cam->fx;
    info.fy = Config::cam->fy;
    info.cx = Config::cam->cx;
    info.cy = Config::cam->cy;

    apriltag_pose_t pose, altPose;
    double err1, err2;
    double err = estimate_tag_pose(&info, &pose, &altPose, &err1, &err2);
    detection.poseAmbiguity = calculate_pose_ambiguity(err1, err2);

    detection.bestPose.pos.populate_from_mat(pose.t);
    detection.bestPose.rot.populate_from_rot_matrix(pose.R);
    matd_destroy(pose.R);
    matd_destroy(pose.t);

    detection.altPose.pos.populate_from_mat(altPose.t);
    detection.altPose.rot.populate_from_rot_matrix(altPose.R);
    matd_destroy(altPose.R);
    matd_destroy(altPose.t);

    // Tag corners
    for(int i = 0; i <= 3; i++) {
      PhotonCompat::Translation2d corner;
      corner.tx = det->p[i][0];
      corner.ty = det->p[i][1];
      detection.tagCorners.push_back(corner);
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
    packet.detections.push_back(detection);
  }
  std::vector<uint8_t> packetBytes = packet.serialize();
  PhotonCompat::PhotonPacket::publish_packet_to_nt(this->ntInst, packetBytes);
  apriltag_detections_destroy(detections);
}

/**
 * Estimate tag pose. Copied from apriltag_pose.c, modified to return err1 and err2.
 */
double ApriltagDetect::estimate_tag_pose(apriltag_detection_info_t* info, apriltag_pose_t* pose, apriltag_pose_t* altPose, double* err1, double* err2) {
    apriltag_pose_t pose1, pose2;
    estimate_tag_pose_orthogonal_iteration(info, err1, &pose1, err2, &pose2, 50);
    if (*err1 <= *err2) {
        pose->R = pose1.R;
        pose->t = pose1.t;
        if (pose2.R) {
            altPose->R = pose2.R;
            altPose->t = pose2.t;
        }
        return *err1;
    } else {
        pose->R = pose2.R;
        pose->t = pose2.t;
        altPose->R = pose1.R;
        altPose->t = pose1.t;
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

double ApriltagDetect::calculate_pose_ambiguity(double err1, double err2) {
  double min = std::min(err1, err2);
  double max = std::max(err1, err2);

  if (max > 0) {
    return min / max;
  } else {
    return -1;
  }
}
