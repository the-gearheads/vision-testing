#pragma once

class ApriltagDetect {
  public:
  ApriltagDetect(json config, NT_Inst ntInst);
  ~ApriltagDetect();
  /* Do detections, draw on the frame*/
  void execute(Mat img);
  /* Currently no-op */
  void reconfigure_detector();

  private:
  json config;
  apriltag_family_t* tag_family;
  apriltag_detector_t* detector;
  NT_Inst ntInst;

  double estimate_tag_pose(apriltag_detection_info_t* info, apriltag_pose_t* pose, double* err1, double* err2);
  double calc_tag_area(apriltag_detection_t* detection);
  std::vector<uint8_t> longToBytes(long long d);
  std::vector<uint8_t> intToBytes(int32_t i);
  template <class T1, class T2> void ContainerInsert(T1& t1, const T2& t2);
};