#include "transport/sender/sender.h"

#include <cassert>

#include "rtp/rtp_handler.h"

RtpSender::RtpSender(SSRCInfo& ssrcInfo) : belongingSSRCInfo_(ssrcInfo) {}

void RtpSender::Enqueue(RtpBizPacket&& rtpBizPacket) {
  // fix: power seq is related to ssrc
  this->sendQueue_.insert(std::move(rtpBizPacket));
  assert(rtpBizPacket.rtpRawPacket.empty());
}

int RtpSender::GetQueueSize() const { return this->sendQueue_.size(); }

// 加入queue时已保证有序和完整
// should pacing
std::vector<RtpBizPacket> RtpSender::Dequeue() {
  std::vector<RtpBizPacket> packets;

  for (auto it = sendQueue_.begin(); it != sendQueue_.end();) {
    // hack: modify set's element, should use C++17 set::extract
    packets.emplace_back(std::move(const_cast<RtpBizPacket&>(*it)));
    assert(it->rtpRawPacket.empty());

    it = sendQueue_.erase(it);
  }

  return packets;
}