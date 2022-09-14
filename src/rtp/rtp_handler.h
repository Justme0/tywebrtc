#ifndef RTP_RTP_HANDLER_H_
#define RTP_RTP_HANDLER_H_

#include <unordered_map>
#include <vector>

#include "rtp/codec_parser/rtp2h264.h"

class PeerConnection;

class RtpHandler {
 public:
  explicit RtpHandler(PeerConnection &pc);

  int HandleRtpPacket(const std::vector<char> &vBufReceive);

 private:
  int HandleRtcpPacket_(const std::vector<char> &vBufReceive);

 private:
  PeerConnection &belongingPeerConnection_;
  std::unordered_map<int, H264Unpacketizer> ssrc2unpacker;
};

#endif  // RTP_RTP_HANDLER_H_
