#pragma once

class CalibrateMode{
  public:
  CalibrateMode(NT_Inst ntInst);
  void execute(Mat img);
  void reset();
};