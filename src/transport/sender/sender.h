#ifndef TRANSPORT_SENDER_SENDER_H_
#define TRANSPORT_SENDER_SENDER_H_

#include <set>
#include <vector>

#include "rtp/rtp_parser.h"

class SSRCInfo;

class RtpSender {
 public:
  explicit RtpSender(SSRCInfo& ssrcInfo);

  void Enqueue(RtpBizPacket&& rtpBizBuffer);
  std::vector<RtpBizPacket> Dequeue();
  int GetQueueSize() const;

 // origin remote ssrc from server
  uint32_t remoteSSRC = 0;
 private:
  SSRCInfo& belongingSSRCInfo_;

  const int kMaxSendQueueLen = 5000;

  // for client NACK, store protected data.
  // OPT: add pacing
  std::map<PowerSeqT, RtpBizPacket> sendQueue_;
};

#endif  //   TRANSPORT_SENDER_SENDER_H_