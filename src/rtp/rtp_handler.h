#ifndef RTP_RTP_HANDLER_H_
#define RTP_RTP_HANDLER_H_

#include <unordered_map>
#include <vector>

#include "rtp/pack_unpack/rtp_to_h264.h"
#include "transport/receiver/receiver.h"
#include "transport/sender/sender.h"

// don't include whole head file avoid recycle reference
class PeerConnection;

struct SSRCInfo {
  SSRCInfo() : rtpReceiver(*this), rtpSender(*this) {}

  // why define ssrc's unpacketizer: save unpacked frame e.g. FU-A
  H264Unpacketizer h264Unpacketizer;
  RtpReceiver rtpReceiver;
  RtpSender rtpSender;

  uint16_t biggestSeq = 0;
  int64_t biggestCycle = 0;
};

class RtpHandler {
 public:
  explicit RtpHandler(PeerConnection &pc);

  int HandleRtpPacket(const std::vector<char> &vBufReceive);

  std::string ToString() const;

 private:
  int HandleRtcpPacket_(const std::vector<char> &vBufReceive);
  int SendToPeer_(std::vector<char> &packet);

 private:
  PeerConnection &belongingPeerConnection_;
  std::unordered_map<uint32_t, SSRCInfo> ssrcInfoMap_;
};

#endif  // RTP_RTP_HANDLER_H_
