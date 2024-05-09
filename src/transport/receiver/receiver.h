#ifndef TRANSPORT_RECEIVER_RECEIVER_H_
#define TRANSPORT_RECEIVER_RECEIVER_H_

#include <set>
#include <vector>

#include "src/rtp/rtp_parser.h"

namespace tywebrtc {

class SSRCInfo;

const PowerSeqT kShitRecvPowerSeqInitValue = -1;

class RtpReceiver {
 public:
  explicit RtpReceiver(SSRCInfo& ssrcInfo);

  void PushToJitter(RtpBizPacket&& rtpBizPacket);
  std::vector<RtpBizPacket> PopOrderedPackets();
  int GetJitterSize() const;

  std::string ToString() const;

 public:
  SSRCInfo& belongingSSRCInfo_;

  // must be ordered, cannot be hashmap
  std::map<PowerSeqT, RtpBizPacket> jitterBuffer_;

  PowerSeqT lastPoppedPowerSeq_ = kShitRecvPowerSeqInitValue;
};

}  // namespace tywebrtc

#endif  //   TRANSPORT_RECEIVER_RECEIVER_H_