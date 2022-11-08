#include "transport/receiver/receiver.h"

#include <cassert>

#include "rtp/rtp_handler.h"

RtpReceiver::RtpReceiver(SSRCInfo& ssrcInfo) : belongingSSRCInfo_(ssrcInfo) {}

// TODO: do NACK
void RtpReceiver::PushToJitter(RtpBizPacket&& rtpBizPacket) {
  jitterBuffer_.insert(std::move(rtpBizPacket));
}

int RtpReceiver::GetJitterSize() const { return jitterBuffer_.size(); }

// OPT: not wait stategy
std::vector<RtpBizPacket> RtpReceiver::PopOrderedPackets() {
  std::vector<RtpBizPacket> orderedPackets;

  for (auto it = jitterBuffer_.begin();
       it != jitterBuffer_.end() &&
       (kShitRecvPowerSeqInitValue == lastPowerSeq_ ||
        it->GetPowerSeq() == lastPowerSeq_ + 1);) {
    tylog("pop from jitter rtp=%s.", it->ToString().data());
    lastPowerSeq_ = it->GetPowerSeq();

    // hack: modify set's element, should use C++17 set::extract
    orderedPackets.emplace_back(std::move(const_cast<RtpBizPacket&>(*it)));
    assert(it->rtpRawPacket.empty());

    it = jitterBuffer_.erase(it);
  }

  if (orderedPackets.empty() && !jitterBuffer_.empty()) {
    tylog(
        "jitter detect out-of-order or lost, cannot out packets, last out "
        "packet powerSeq=%ld, jitter.size=%zu (first rtp=%s).",
        lastPowerSeq_, jitterBuffer_.size(),
        jitterBuffer_.begin()->ToString().data());
  }

  return orderedPackets;
}
