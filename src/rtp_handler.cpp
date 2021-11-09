#include "rtp_handler.h"

RTPHandler::RTPHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

int RTPHandler::HandleRtpPacket(const std::vector<char> &vBufReceive) {
  return 0;
}
