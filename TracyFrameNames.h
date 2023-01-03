#pragma once
#define ADD_FRAME_NAME_CPP(name, text) const char* const name = text;
#define ADD_FRAME_NAME_H(name) extern const char* const name;

ADD_FRAME_NAME_H(tcy_grabbing_video_frame)
ADD_FRAME_NAME_H(tcy_apriltag_detect)
ADD_FRAME_NAME_H(tcy_apriltag_pose_est)
ADD_FRAME_NAME_H(tcy_opencv_bgr2gray)
ADD_FRAME_NAME_H(tcy_render_meter)
ADD_FRAME_NAME_H(tcy_opencv_videowriter_write)