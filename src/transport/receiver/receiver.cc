// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/transport/receiver/receiver.h"

#include <cassert>

#include "src/pc/peer_connection.h"

namespace tywebrtc {

RtpReceiver::RtpReceiver(SSRCInfo& ssrcInfo)
    : belongingSSRCInfo_(ssrcInfo),
      receiverReportTimer_(*this),
      pliTimer_(*this) {
  assert(rtpStats_.probation == 0);
}

void RtpReceiver::PushToJitter(RtpBizPacket&& rtpBizPacket) {
  rtpBizPacket.enterJitterTimeMs = g_now_ms;
  jitterBuffer_.emplace(rtpBizPacket.GetPowerSeq(), std::move(rtpBizPacket));
  assert(rtpBizPacket.rtpRawPacket.empty());

  if (!is_add_rr_timer_) {
    TimerManager::Instance()->AddTimer(&receiverReportTimer_);
    is_add_rr_timer_ = true;
  }

  if (!belongingSSRCInfo_.is_audio_ && !is_add_pli_timer_) {
    TimerManager::Instance()->AddTimer(&pliTimer_);
    is_add_pli_timer_ = true;
  }
}

int RtpReceiver::GetJitterSize() const { return jitterBuffer_.size(); }

std::string RtpReceiver::ToString() const {
  return tylib::format_string("{jitterSize=%zu, lastPowerSeq=%s, rtpStats=%s}",
                              jitterBuffer_.size(),
                              PowerSeqToString(lastPoppedPowerSeq_).data(),
                              rtpStats_.ToString().data());
}

// OPT: use not wait stategy
std::vector<RtpBizPacket> RtpReceiver::PopOrderedPackets() {
  std::vector<RtpBizPacket> orderedPackets;

  for (auto it = jitterBuffer_.begin();
       it != jitterBuffer_.end() &&
       (kShitRecvPowerSeqInitValue == lastPoppedPowerSeq_ ||
        it->first == lastPoppedPowerSeq_ + 1);
       it = jitterBuffer_.erase(it)) {
    tylog("pop from jitter rtp=%s.", it->second.ToString().data());
    lastPoppedPowerSeq_ = it->first;

    orderedPackets.emplace_back(std::move(it->second));
    assert(it->second.rtpRawPacket.empty());
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

    const uint32_t kMediaSrcSSRC =
        bAudioType ? this->belongingSSRCInfo_.belongingRtpHandler.upAudioSSRC
                   : this->belongingSSRCInfo_.belongingRtpHandler.upVideoSSRC;
    assert(0 != kMediaSrcSSRC &&
           kMediaSrcSSRC == firstRtpHeader.getSSRC());  // already recv

    int ret = this->belongingSSRCInfo_.belongingRtpHandler.belongingPC_
                  .rtcpHandler_.rtpfb_.nack_.CreateNackSend(
                      nackSeqs, kSelfRtcpSSRC, kMediaSrcSSRC);
    if (ret) {
      tylog("createNackReportSend ret=%d", ret);

      return {};
    }

    return {};
  }

  // wait too long for audio, pop all
  if (bAudioType) {
    tylog(
        "audio wait too long %ldms or jitterSize=%zu too long, not wait, pop "
        "all",
        waitMs, jitterBuffer_.size());
    // OPT: should pop only first, and pop remaining in normal way
    for (auto it = jitterBuffer_.begin(); it != jitterBuffer_.end();) {
      tylog("pop from jitter rtp=%s.", it->second.ToString().data());
      lastPoppedPowerSeq_ = it->first;

      orderedPackets.emplace_back(std::move(it->second));
      assert(it->second.rtpRawPacket.empty());

      it = jitterBuffer_.erase(it);
    }

    return orderedPackets;
  } else {
    assert(!jitterBuffer_.empty());

    // wait too long for video, clear and do PLI
    tylog("PLI, first packet waitMs=%ld or jitterSize=%zu too long, packet=%s.",
          waitMs, jitterBuffer_.size(), firstPacket.ToString().data());

    int ret = belongingSSRCInfo_.belongingRtpHandler.belongingPC_.rtcpHandler_
                  .psfb_.pli_.CreatePLISend();
    if (ret) {
      tylog("createPLIReportSend ret=%d", ret);
      // not return
    }

    // OPT: discard P frame, recognize I frame and pop
    lastPoppedPowerSeq_ = (--jitterBuffer_.end())->first;
    tylog("pop all pkt in jitter=%s, set lastPoppedPowerSeq=%ld.",
          tylib::AnyToString(jitterBuffer_).data(), lastPoppedPowerSeq_);
    jitterBuffer_.clear();

    // OPT: next recv pkt, check and only recv I frame

    return {};
  }
}

}  // namespace tywebrtc
