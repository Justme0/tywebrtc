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

  // int GetQueueSize() const;
  const std::vector<char>* GetSeqPacket(PowerSeqT powerSeq) const;

 private:
  SSRCInfo& belongingSSRCInfo_;

  std::map<PowerSeqT, RtpBizPacket> sendQueue_;

  // [begin, saveLast_] is already sent, saved for downlink NACK, len <=
  // kSendQueueSaveLen
  const size_t kSendQueueSaveLen = 1000;

  // must be initialized to map end(), and defined after sendQueue_, see
  // constructor
  std::map<PowerSeqT, RtpBizPacket>::const_iterator saveLast_;
};

#endif  //   TRANSPORT_SENDER_SENDER_H_