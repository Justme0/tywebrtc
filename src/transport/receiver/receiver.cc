#include "transport/receiver/receiver.h"

#include <cassert>

#include "pc/peer_connection.h"
#include "rtp/rtcp/rtcp_nack.h"
#include "rtp/rtp_handler.h"

RtpReceiver::RtpReceiver(SSRCInfo& ssrcInfo) : belongingSSRCInfo_(ssrcInfo) {}

// TODO: do NACK
void RtpReceiver::PushToJitter(RtpBizPacket&& rtpBizPacket) {
  rtpBizPacket.enterJitterTimeMs = g_now_ms;
  jitterBuffer_.emplace(rtpBizPacket.GetPowerSeq(), std::move(rtpBizPacket));
  assert(rtpBizPacket.rtpRawPacket.empty());
}

int RtpReceiver::GetJitterSize() const { return jitterBuffer_.size(); }

std::string RtpReceiver::ToString() const {
  return tylib::format_string("{jitterSize=%zu, lastPowerSeq=%s}",
                              jitterBuffer_.size(),
                              PowerSeqToString(lastPowerSeq_).data());
}

// OPT: not wait stategy
std::vector<RtpBizPacket> RtpReceiver::PopOrderedPackets() {
  std::vector<RtpBizPacket> orderedPackets;

  for (auto it = jitterBuffer_.begin();
       it != jitterBuffer_.end() &&
       (kShitRecvPowerSeqInitValue == lastPowerSeq_ ||
        it->first == lastPowerSeq_ + 1);) {
    tylog("pop from jitter rtp=%s.", it->second.ToString().data());
    lastPowerSeq_ = it->first;

    orderedPackets.emplace_back(std::move(it->second));
    assert(it->second.rtpRawPacket.empty());

    it = jitterBuffer_.erase(it);
  }

  // OPT: move constant to config
  if (jitterBuffer_.size() > 10000) {
    std::string err = tylib::format_string(
        "bug, jitterBuffer_ size too large=%zu.", jitterBuffer_.size());
    tylog("%s", err.data());
    assert(!err.data());
  }

  tylog("orderedPackets size=%zu, jitter size=%zu.", orderedPackets.size(),
        jitterBuffer_.size());
  if (orderedPackets.empty() && !jitterBuffer_.empty()) {
    const RtpBizPacket& firstPacket = jitterBuffer_.begin()->second;
    tylog(
        "jitter detect out-of-order or lost, cannot out packets, last out "
        "packet powerSeq=%ld[%s], jitter.size=%zu, jitter first rtp=%s.",
        lastPowerSeq_, PowerSeqToString(lastPowerSeq_).data(),
        jitterBuffer_.size(), firstPacket.ToString().data());

    const int64_t waitMs = firstPacket.WaitTimeMs();

    // 2s is for debug
    if (waitMs > 2 * 1000) {
      g_bugShutDown->Add({{"dummy", "fail"}}).Increment();

      std::string err =
          tylib::format_string("bug, first waitMs=%ld too long, packet=%s.",
                               waitMs, firstPacket.ToString().data());
      tylog("%s", err.data());
      assert(!err.data());
    }

    // const int64_t kRtcpRoundTripTimeMs = 40;

    // OPT: move constant to config, related to RTT
    const int64_t kNackTimeMs = 0;
    // const int64_t kPLITimeMs = kRtcpRoundTripTimeMs + 5;
    if (waitMs < kNackTimeMs) {
      tylog(
          "We wait short time %ldms, do nothing. Sometime not real lost, just "
          "out-of-order.",
          waitMs);
    } else if (kNackTimeMs <= waitMs /*&& waitMs < kPLITimeMs*/) {
      tylog("wait %ldms, to nack", waitMs);
      std::set<int> nackSeqs;
      for (int i = lastPowerSeq_ + 1; i < firstPacket.GetPowerSeq(); ++i) {
        nackSeqs.insert(SplitPowerSeq(i).second);
      }

      const int kSelfRtcpSSRC = 1;
      int ret = this->belongingSSRCInfo_.belongingRtpHandler
                    .belongingPeerConnection_.rtcpHandler_.CreateNackReportSend(
                        nackSeqs, kSelfRtcpSSRC,
                        reinterpret_cast<const RtpHeader*>(
                            firstPacket.rtpRawPacket.data())
                                    ->GetMediaType() == kMediaTypeAudio
                            ? g_UplinkAudioSsrc
                            : g_UplinkVideoSsrc);

      if (ret) {
        tylog("createNackReportSend ret=%d", ret);
        // not return error
        // should monitor
      }
    } else {
      // const RtpHeader& rtpHeader = *reinterpret_cast<const
      // RtpHeader*>(firstPacket.rtpRawPacket.data());

      // if (rtpHeader.GetMediaType() == kMediaTypeAudio) {
      // tylog("wait %ldms, not wait, pop all", waitMs);
      // for (auto it = jitterBuffer_.begin(); it != jitterBuffer_.end();) {
      //   tylog("pop from jitter rtp=%s.", it->ToString().data());
      //   lastPowerSeq_ = it->GetPowerSeq();

      //   // hack: modify set's element, should use C++17 set::extract,
      //   // now we don't move key member
      //   orderedPackets.emplace_back(std::move(const_cast<RtpBizPacket&>(*it)));
      //   assert(it->rtpRawPacket.empty());

      //   it = jitterBuffer_.erase(it);
      // }
      // } else {
      // assert(rtpHeader.GetMediaType() == kMediaTypeVideo);

      // todo: if not receive I frame some time after beginning, should req
      // tylog("PLI");
      // }
    }
  }

  return orderedPackets;
}
