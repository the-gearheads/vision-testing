#pragma once

class ApriltagDetect {
  public:
  ApriltagDetect(NT_Inst ntInst);
  ~ApriltagDetect();
  /* Do detections, draw on the frame*/
  void execute(Mat img);
  void reset() {};
  private:
  apriltag_family_t* tag_family;
  apriltag_detector_t* detector;
  NT_Inst ntInst;
};