#pragma once

#include <string>

class PeerConnection;

class SrtpHandler {
 public:
  PeerConnection &belongingPeerConnection_;

  explicit SrtpHandler(PeerConnection &pc);

 public:
  std::string encryptKey_;  // when send rtp
  std::string decryptKey_;  // when recv srtp
};
