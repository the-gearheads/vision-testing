#pragma once

class ApriltagDetect {
  public:
  ApriltagDetect(json config);
  ~ApriltagDetect();
  /* Do detections, draw on the frame*/
  void execute(Mat img);
  /* Currently no-op */
  void reconfigure_detector();

  private:
  json config;
  apriltag_family_t* tag_family;
  apriltag_detector_t* detector;
};