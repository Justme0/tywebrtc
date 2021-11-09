#pragma once

#include <vector>

class PeerConnection;

class RTPHandler {
 public:
  explicit RTPHandler(PeerConnection &pc);

  int HandleRtpPacket(const std::vector<char> &vBufReceive);

  // private:
  PeerConnection &belongingPeerConnection_;
};
