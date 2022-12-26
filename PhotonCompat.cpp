#include "PhotonCompat.h"
#include "Config.h"

//#define ENCODING_DEBUG

/* Adds the contents of vec2 to the end of vec1. Needs to be like this due to how macros work */
#define ADD_VECTOR(vec1, vec2) \
  { \
    auto v = vec2;\
    vec1.insert(vec1.end(), v.begin(), v.end()); \
  }

void PhotonCompat::encodeDouble(double src, std::vector<uint8_t>& packetData) {
    uint64_t data = *reinterpret_cast<uint64_t*>(&src);
    packetData.push_back(static_cast<uint8_t>((data >> 56) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 48) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 40) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 32) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 24) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 16) & 0xff));
    packetData.push_back(static_cast<uint8_t>((data >> 8) & 0xff));
    packetData.push_back(static_cast<uint8_t>(data & 0xff));

    #ifdef ENCODING_DEBUG
    printf("[double] Encoding %f as ", src);
    for(int i=packetData.size()-8; i < packetData.size(); i++) {
      printf("%02x ", packetData.at(i));
    }
    printf("\n");
    #endif

}


void PhotonCompat::encodeInt(int src, std::vector<uint8_t>& packetData) {
    packetData.push_back(static_cast<uint8_t>(src >> 24));
    packetData.push_back(static_cast<uint8_t>(src >> 16));
    packetData.push_back(static_cast<uint8_t>(src >> 8));
    packetData.push_back(static_cast<uint8_t>(src));
    #ifdef ENCODING_DEBUG
    printf("[int] Encoding %d as ", src);
    for(int i=packetData.size()-4; i < packetData.size(); i++) {
      printf("%02x ", packetData.at(i));
    }
    printf("\n");
    #endif
}

void PhotonCompat::encodeByte(uint8_t src, std::vector<uint8_t>& packetData) {
    packetData.push_back(static_cast<uint8_t>(src));
    #ifdef ENCODING_DEBUG
    printf("[byte] Encoding %d as %02x\n", src, src);
    #endif
}

std::vector<uint8_t> PhotonCompat::Translation3d::serialize() {
  std::vector<uint8_t> out;
  encodeDouble(this->tx, out);
  encodeDouble(this->ty, out);
  encodeDouble(this->tz, out);
  return out;
}

std::vector<uint8_t> PhotonCompat::Translation2d::serialize() {
  std::vector<uint8_t> out;
  encodeDouble(this->tx, out);
  encodeDouble(this->ty, out);
  return out;
}

void PhotonCompat::Rotation3d::populate_from_rot_matrix(matd_t* rot) {
  // I sure hope this works, this code is AI generated because i do not understand this math
  // Anyways, it is supposed to convert the pose rotation matrix to a quaternion
  this->qw = sqrt(1 + MATD_EL(rot, 0, 0) + MATD_EL(rot, 1, 1) + MATD_EL(rot, 2, 2)) / 2;
  this->qx = (MATD_EL(rot, 2, 1) - MATD_EL(rot, 1, 2)) / (4 * qw);
  this->qy = (MATD_EL(rot, 0, 2) - MATD_EL(rot, 2, 0)) / (4 * qw);
  this->qz = (MATD_EL(rot, 1, 0) - MATD_EL(rot, 0, 1)) / (4 * qw);
}

std::vector<uint8_t> PhotonCompat::Rotation3d::serialize() {
  std::vector<uint8_t> out;
  encodeDouble(this->qw, out);
  encodeDouble(this->qx, out);
  encodeDouble(this->qy, out);
  encodeDouble(this->qz, out);
  return out;
}

std::vector<uint8_t> PhotonCompat::Transform::serialize() {
  std::vector<uint8_t> out;
  ADD_VECTOR(out, this->pos.serialize());
  ADD_VECTOR(out, this->rot.serialize());
  return out;
}

std::vector<uint8_t> PhotonCompat::PhotonDetection::serialize() {
  std::vector<uint8_t> out;
  encodeDouble(this->yaw, out);
  encodeDouble(this->pitch, out);
  encodeDouble(this->skew, out);
  encodeDouble(this->area, out);
  encodeInt(this->tagId, out);
  ADD_VECTOR(out, this->pose.serialize());
  encodeDouble(this->poseAmbiguity, out);

  for(Translation2d t: this->tagCorners) {
    ADD_VECTOR(out, t.serialize());
  }

  return out;
}

int heartbeat = 0;
void PhotonCompat::PhotonPacket::publish_packet_to_nt(NT_Inst nt, std::vector<uint8_t> packet) {
  std::basic_string_view sv(reinterpret_cast<char*>(packet.data()), packet.size());

  nt::SetEntryTypeValue(nt::GetEntry(nt, Config::nt->fullPath+"/rawBytes"), nt::Value::MakeRaw(sv));
  heartbeat++;
  nt::SetEntryTypeValue(nt::GetEntry(nt, Config::nt->fullPath+"/heartbeat"), nt::Value::MakeDouble(heartbeat));
  nt::SetEntryTypeValue(nt::GetEntry(nt, Config::nt->rootPrefix+"/version"), nt::Value::MakeString(Config::nt->reportPhotonVersion));
}

std::vector<uint8_t> PhotonCompat::PhotonPacket::serialize() {
  std::vector<uint8_t> out;
  encodeDouble(this->latency, out);
  encodeByte(this->detections.size(), out);
  for(auto det: this->detections) {
    ADD_VECTOR(out, det.serialize());
  }

  #ifdef ENCODING_DEBUG
  if(true) { 
    for(int i=0; i < out.size(); i++) {
      printf("%02x ", out.at(i));
    }
    printf("\n");
  }
  #endif

  return out;
}