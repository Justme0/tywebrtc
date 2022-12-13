#ifndef TRANSPORT_RECEIVER_RECEIVER_H_
#define TRANSPORT_RECEIVER_RECEIVER_H_

#include <set>
#include <vector>

#include "rtp/rtp_parser.h"

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

  // mark last popped one, should reserve rtp for nack,
  PowerSeqT lastPowerSeq_ = kShitRecvPowerSeqInitValue;
};

#endif  //   TRANSPORT_RECEIVER_RECEIVER_H_