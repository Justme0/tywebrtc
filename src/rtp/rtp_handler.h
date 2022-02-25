#ifndef RTP_RTP_HANDLER_H_
#define RTP_RTP_HANDLER_H_

#include <vector>

class PeerConnection;

class RtpHandler {
 public:
  explicit RtpHandler(PeerConnection &pc);

  int HandleRtpPacket(const std::vector<char> &vBufReceive);

 private:
  int HandleRtcpPacket_(const std::vector<char> &vBufReceive);

 private:
  PeerConnection &belongingPeerConnection_;
};

#endif  // RTP_RTP_HANDLER_H_
