#include "transport/receiver/receiver.h"

#include <cassert>

#include "rtp/rtcp/rtcp_nack.h"
#include "rtp/rtp_handler.h"

RtpReceiver::RtpReceiver(SSRCInfo& ssrcInfo) : belongingSSRCInfo_(ssrcInfo) {}

// TODO: do NACK
void RtpReceiver::PushToJitter(RtpBizPacket&& rtpBizPacket) {
  rtpBizPacket.enterJitterTimeMs = g_now_ms;
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

    // hack: modify set's element, should use C++17 set::extract,
    // now we don't move key member
    orderedPackets.emplace_back(std::move(const_cast<RtpBizPacket&>(*it)));
    assert(it->rtpRawPacket.empty());

    it = jitterBuffer_.erase(it);
  }

  // OPT: move constant to config
  if (jitterBuffer_.size() > 10000) {
    std::string err = tylib::format_string(
        "bug, jitterBuffer_ size too large=%zu.", jitterBuffer_.size());
    tylog("%s", err.data());
    assert(!err.data());
  }

  if (orderedPackets.empty() && !jitterBuffer_.empty()) {
    const RtpBizPacket& firstPacket = *jitterBuffer_.begin();
    tylog(
        "jitter detect out-of-order or lost, cannot out packets, last out "
        "packet powerSeq=%ld[%s], jitter.size=%zu, jitter first rtp=%s.",
        lastPowerSeq_, PowerSeqToString(lastPowerSeq_).data(),
        jitterBuffer_.size(), firstPacket.ToString().data());

    const int64_t waitMs = firstPacket.WaitTimeMs();

    if (waitMs > 5 * 1000) {
      std::string err =
          tylib::format_string("bug, first waitMs=%ld too long, packet=%s.",
                               waitMs, firstPacket.ToString().data());
      tylog("%s", err.data());
      assert(!err.data());
    }

    // OPT: move constant to config, related to RTT
    const int64_t kNackTimeMs = -1;
    const int64_t kPLITimeMs = -1;
    if (waitMs < kNackTimeMs) {
      tylog(
          "We wait short time %ldms, do nothing. Sometime not real lost, just "
          "out-of-order.",
          waitMs);
    } else if (kNackTimeMs <= waitMs && waitMs < kPLITimeMs) {
      tylog("wait %ldms, to nack", waitMs);
      std::set<int> nackSeqs;
      for (int i = lastPowerSeq_ + 1; i < firstPacket.GetPowerSeq(); ++i) {
        nackSeqs.insert(SplitPowerSeq(i).second);
      }
      std::vector<char> nackPacket;
      // taylor arg
      CreateNackReport(nackSeqs, 0, 0, nackPacket);
      // OPT: to nack, now not do
    } else {
      // const RtpHeader& rtpHeader = *reinterpret_cast<const
      // RtpHeader*>(firstPacket.rtpRawPacket.data());

      // if (rtpHeader.GetMediaType() == kMediaTypeAudio) {
      tylog("wait %ldms, not wait, pop all", waitMs);
      for (auto it = jitterBuffer_.begin(); it != jitterBuffer_.end();) {
        tylog("pop from jitter rtp=%s.", it->ToString().data());
        lastPowerSeq_ = it->GetPowerSeq();

        // hack: modify set's element, should use C++17 set::extract,
        // now we don't move key member
        orderedPackets.emplace_back(std::move(const_cast<RtpBizPacket&>(*it)));
        assert(it->rtpRawPacket.empty());

        it = jitterBuffer_.erase(it);
      }
      // } else {
      // assert(rtpHeader.GetMediaType() == kMediaTypeVideo);

      // todo: if not receive I frame some time after beginning, should req
      // tylog("PLI");
      // }
    }
  }

  return orderedPackets;
}
