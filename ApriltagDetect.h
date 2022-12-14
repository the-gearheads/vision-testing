#pragma once

class ApriltagDetect {
  public:
  ApriltagDetect(json config, NT_Inst ntInst);
  ~ApriltagDetect();
  /* Do detections, draw on the frame*/
  void execute(Mat img, double lastLatencyVal);
  /* Currently no-op */
  void reconfigure_detector();

  private:
  unsigned int heartbeat = 0;
  json config;
  apriltag_family_t* tag_family;
  apriltag_detector_t* detector;
  NT_Inst ntInst;

  double estimate_tag_pose(apriltag_detection_info_t* info, apriltag_pose_t* pose, apriltag_pose_t* altPose, double* err1, double* err2);
  double calculate_pose_ambiguity(double err1, double err2);
  double calc_tag_area(apriltag_detection_t* detection);
};