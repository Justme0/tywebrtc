#ifndef RTP_RTP_HANDLER_H_
#define RTP_RTP_HANDLER_H_

#include <unordered_map>
#include <vector>

#include "rtp/pack_unpack/rtp_to_h264.h"

class PeerConnection;

class RtpHandler {
 public:
  explicit RtpHandler(PeerConnection &pc);

  int HandleRtpPacket(const std::vector<char> &vBufReceive);

  std::string ToString() const;

 private:
  int HandleRtcpPacket_(const std::vector<char> &vBufReceive);

 private:
  PeerConnection &belongingPeerConnection_;
  // why define ssrc map: save unpacked frame e.g. FU-A
  std::unordered_map<uint32_t, H264Unpacketizer> ssrc2unpacker_;
};

#endif  // RTP_RTP_HANDLER_H_
