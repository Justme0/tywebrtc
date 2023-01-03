#ifndef TRANSPORT_SENDER_SENDER_H_
#define TRANSPORT_SENDER_SENDER_H_

#include <set>
#include <vector>

#include "rtp/rtp_parser.h"

class SSRCInfo;

// audio should not in pacing
class RtpSender {
 public:
  explicit RtpSender(SSRCInfo& ssrcInfo);

  void Enqueue(RtpBizPacket&& rtpBizPacket);
  std::vector<RtpBizPacket> Dequeue();
  int GetQueueSize() const;

 private:
  SSRCInfo& belongingSSRCInfo_;
  std::map<PowerSeqT, RtpBizPacket> sendQueue_;
};

#endif  //   TRANSPORT_SENDER_SENDER_H_