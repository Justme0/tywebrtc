#include "rtp_handler.h"

#include <string>

RTPHandler::RTPHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

std::string g_anotherServerIp = "127.0.0.1";

int RTPHandler::HandleRtpPacket(const std::vector<char> &vBufReceive) {
  return 0;
}
