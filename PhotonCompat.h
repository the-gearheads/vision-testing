#pragma once

#define E_EMPTYFIELD -1000

namespace PhotonCompat {
  class Translation2d {
    public:
    double tx = E_EMPTYFIELD;
    double ty = E_EMPTYFIELD;
    std::vector<uint8_t> serialize();
  };

  class Translation3d {
    public:
    double tx = E_EMPTYFIELD;
    double ty = E_EMPTYFIELD;
    double tz = E_EMPTYFIELD;
    std::vector<uint8_t> serialize();
  };

  class Rotation3d {
    public:
    double qx = E_EMPTYFIELD;
    double qy = E_EMPTYFIELD;
    double qz = E_EMPTYFIELD;
    double qw = E_EMPTYFIELD;
    std::vector<uint8_t> serialize();
    void populate_from_rot_matrix(matd_t* rot);
  };

  class Transform {
    public:
    Translation3d pos;
    Rotation3d rot;
    std::vector<uint8_t> serialize();
  };

  class PhotonDetection {
    public:
    int tagId = E_EMPTYFIELD;
    double yaw = E_EMPTYFIELD;
    double pitch = E_EMPTYFIELD;
    double area = E_EMPTYFIELD;
    double skew = E_EMPTYFIELD;
    Transform pose;
    double poseAmbiguity;
    std::vector<Translation2d> tagCorners;
    std::vector<uint8_t> serialize();
  };

  class PhotonPacket {
    public:
    double latency = E_EMPTYFIELD;
    std::vector<PhotonDetection> detections;
    std::vector<uint8_t> serialize();
    static void publish_packet_to_nt(NT_Inst nt, std::vector<uint8_t> packetData);
  };

  void encodeDouble(double src, std::vector<uint8_t>& packetData);
  void encodeInt(int src, std::vector<uint8_t>& packetData);
  void encodeByte(uint8_t src, std::vector<uint8_t>& packetData);
}