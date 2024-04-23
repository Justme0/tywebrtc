#include "src/transport/receiver/receiver.h"

#include <cassert>

#include "src/pc/peer_connection.h"
#include "src/rtp/rtcp/rtcp_packet/rtp_fb/rtcp_nack.h"
#include "src/rtp/rtp_handler.h"

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
                              PowerSeqToString(lastPoppedPowerSeq_).data());
}

// OPT: use not wait stategy
std::vector<RtpBizPacket> RtpReceiver::PopOrderedPackets() {
  std::vector<RtpBizPacket> orderedPackets;

  for (auto it = jitterBuffer_.begin();
       it != jitterBuffer_.end() &&
       (kShitRecvPowerSeqInitValue == lastPoppedPowerSeq_ ||
        it->first == lastPoppedPowerSeq_ + 1);) {
    tylog("pop from jitter rtp=%s.", it->second.ToString().data());
    lastPoppedPowerSeq_ = it->first;

    orderedPackets.emplace_back(std::move(it->second));
    assert(it->second.rtpRawPacket.empty());

    it = jitterBuffer_.erase(it);
  }

  // OPT: move constant to config
  if (jitterBuffer_.size() >= 10000) {
    std::string err = tylib::format_string(
        "bug, jitterBuffer_ size too large=%zu.", jitterBuffer_.size());
    tylog("%s", err.data());
    assert(!"jitter buffer size too large");
  }

  tylog("orderedPackets size=%zu, jitter size=%zu.", orderedPackets.size(),
        jitterBuffer_.size());

  if (!orderedPackets.empty()) {
    return orderedPackets;
  }

  assert(!jitterBuffer_.empty());  // already push to jitter

  const RtpBizPacket& firstPacket = jitterBuffer_.begin()->second;
  const RtpHeader& firstRtpHeader =
      *reinterpret_cast<const RtpHeader*>(firstPacket.rtpRawPacket.data());
  const bool bAudioType = firstRtpHeader.ComputeMediaType() == kMediaTypeAudio;
  tylog(
      "jitter detect out-of-order or lost, cannot out packets, last out "
      "packet powerSeq=%ld[%s], jitter.size=%zu, jitter first rtp=%s.",
      lastPoppedPowerSeq_, PowerSeqToString(lastPoppedPowerSeq_).data(),
      jitterBuffer_.size(), firstPacket.ToString().data());

  const int64_t waitMs = firstPacket.WaitTimeMs();

  // const int64_t kRtcpRoundTripTimeMs = 40;

  // OPT: move constant to config, related to RTT
  const int64_t kNackTimeMs = 0;
  // const int64_t kPLITimeMs = kRtcpRoundTripTimeMs + 5;
  const int64_t kPLITimeMs = 300;
  // buffer packet len, OPT: distinguish audio and video
  const size_t kJitterMaxSize = 15;

  if (waitMs < kNackTimeMs) {
    tylog(
        "We wait short time %ldms, do nothing. Sometime not real lost, just "
        "out-of-order.",
        waitMs);

    return {};
  }

  if (kNackTimeMs <= waitMs && waitMs < kPLITimeMs &&
      jitterBuffer_.size() <= kJitterMaxSize) {
    tylog("wait %ldms, to nack", waitMs);
    std::set<int> nackSeqs;
    // should req all not received packets in jitter
    for (int i = lastPoppedPowerSeq_ + 1; i < firstPacket.GetPowerSeq(); ++i) {
      nackSeqs.insert(SplitPowerSeq(i).second);
    }
    tylog("uplink this=%s, firstPacket=%s, nack seqs=%s.", ToString().data(),
          firstPacket.ToString().data(), tylib::AnyToString(nackSeqs).data());
    assert(!nackSeqs.empty());

    const uint32_t kSelfRtcpSSRC = 1;
    const uint32_t kMediaSrcSSRC =
        bAudioType ? this->belongingSSRCInfo_.belongingRtpHandler.upAudioSSRC
                   : this->belongingSSRCInfo_.belongingRtpHandler.upVideoSSRC;
    assert(0 != kMediaSrcSSRC &&
           kMediaSrcSSRC == firstRtpHeader.getSSRC());  // already recv

    int ret = this->belongingSSRCInfo_.belongingRtpHandler
                  .belongingPeerConnection_.rtcpHandler_.nack.CreateNackSend(
                      nackSeqs, kSelfRtcpSSRC, kMediaSrcSSRC);

    if (ret) {
      tylog("createNackReportSend ret=%d", ret);

      return {};
    }

    return {};
  }

  // wait too long for audio, pop all
  if (bAudioType) {
    tylog("audio wait too long %ldms, not wait, pop all", waitMs);
    // OPT: should pop only first, and pop remaining in normal way
    for (auto it = jitterBuffer_.begin(); it != jitterBuffer_.end();) {
      tylog("pop from jitter rtp=%s.", it->second.ToString().data());
      lastPoppedPowerSeq_ = it->first;

      orderedPackets.emplace_back(std::move(it->second));
      assert(it->second.rtpRawPacket.empty());

      it = jitterBuffer_.erase(it);
    }
    return orderedPackets;
  }

  // wait too long for video, clear and do PLI
  tylog("PLI, first packet waitMs=%ld too long, packet=%s.", waitMs,
        firstPacket.ToString().data());

  const uint32_t kSelfRtcpSSRC = 1;
  const uint32_t kMediaSrcSSRC =
      belongingSSRCInfo_.belongingRtpHandler.upVideoSSRC;
  assert(0 != kMediaSrcSSRC);
  int ret = belongingSSRCInfo_.belongingRtpHandler.belongingPeerConnection_
                .rtcpHandler_.pli.CreatePLISend(kSelfRtcpSSRC, kMediaSrcSSRC);
  if (ret) {
    tylog("createPLIReportSend ret=%d", ret);
    // not return
  }

  jitterBuffer_.clear();
  lastPoppedPowerSeq_ = kShitRecvPowerSeqInitValue;

  return {};
}
