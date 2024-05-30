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

#define RTP_SEQ_MOD (1 << 16)

/*
 * Called whenever there is a large jump in sequence numbers,
 * or when they get out of probation...
 */
// only called by rtp_valid_packet_in_sequence
static void rtp_init_sequence(RTPStatistics* s, uint16_t seq) {
  s->max_seq = seq;
  s->cycles = 0;
  s->base_seq = seq - 1;
  s->bad_seq = RTP_SEQ_MOD + 1;
  s->received = 0;
  s->expected_prior = 0;
  s->received_prior = 0;
  s->jitter = 0;
  s->transit = 0;
}

[[maybe_unused]] static void rtcp_update_jitter(RTPStatistics* s,
                                                uint32_t sent_timestamp,
                                                uint32_t arrival_timestamp) {
  // Most of this is pretty straight from RFC 3550 appendix A.8
  uint32_t transit = arrival_timestamp - sent_timestamp;
  uint32_t prev_transit = s->transit;
  int32_t d = transit - prev_transit;
  // Doing the FFABS() call directly on the "transit - prev_transit"
  // expression doesn't work, since it's an unsigned expression. Doing the
  // transit calculation in unsigned is desired though, since it most
  // probably will need to wrap around.
  d = FFABS(d);
  s->transit = transit;
  if (!prev_transit) return;
  s->jitter += d - (int32_t)((s->jitter + 8) >> 4);
}

/* Returns 1 if we should handle this packet. */
bool RtpReceiver::rtp_valid_packet_in_sequence(RTPStatistics* s, uint16_t seq) {
  uint16_t udelta = seq - s->max_seq;
  const int MAX_DROPOUT = 3000;
  const int MAX_MISORDER = 100;
  const int MIN_SEQUENTIAL = 2;

  /* source not valid until MIN_SEQUENTIAL packets with sequence
   * seq. numbers have been received */
  if (s->probation) {
    if (seq == s->max_seq + 1) {
      s->probation--;
      s->max_seq = seq;
      if (s->probation == 0) {
        rtp_init_sequence(s, seq);
        s->received++;
        return 1;
      }
    } else {
      s->probation = MIN_SEQUENTIAL - 1;
      s->max_seq = seq;
    }
  } else if (udelta < MAX_DROPOUT) {
    // in order, with permissible gap
    if (seq < s->max_seq) {
      // sequence number wrapped; count another 64k cycles
      s->cycles += RTP_SEQ_MOD;
    }
    s->max_seq = seq;
  } else if (udelta <= RTP_SEQ_MOD - MAX_MISORDER) {
    // sequence made a large jump...
    tylog("seq=%u, udelta[%u]<=%d, s=%s.", seq, udelta,
          RTP_SEQ_MOD - MAX_MISORDER, s->ToString().data());
    if (seq == s->bad_seq) {
      /* two sequential packets -- assume that the other side
       * restarted without telling us; just resync. */
      rtp_init_sequence(s, seq);
    } else {
      s->bad_seq = (seq + 1) & (RTP_SEQ_MOD - 1);
      return 0;
    }
  } else {
    // duplicate or reordered packet...
  }
  s->received++;
  return 1;
}

RtpReceiver::RtpReceiver(SSRCInfo& ssrcInfo) : belongingSSRCInfo_(ssrcInfo) {
  assert(rtpStats_.probation == 1);
}

void RtpReceiver::CountStatistics(const RtpBizPacket& rtpBizPacket) {
  rtpStats_.octet_count += rtpBizPacket.rtpRawPacket.size();

  /* TODO: I think this is way too often; RFC 1889 has algorithm for this */
  /* XXX: MPEG pts hardcoded. RTCP send every 0.5 seconds */

  /* RTCP packets use 0.5% of the bandwidth */
  const int RTCP_TX_RATIO_NUM = 5;
  const int RTCP_TX_RATIO_DEN = 1000;
  int rtcp_bytes = ((rtpStats_.octet_count - rtpStats_.last_octet_count) *
                    RTCP_TX_RATIO_NUM) /
                   RTCP_TX_RATIO_DEN;

  // mmu_man: that's enough for me... VLC sends much less btw !?
  rtcp_bytes /= 50;
  if (rtcp_bytes < 28) {
    // return;
  };

#ifdef SHIITTT
  avio_wb32(pb,
            fraction); /* 8 bits of fraction, 24 bits of total packets lost */
  avio_wb32(pb, extended_max);          /* max sequence received */
  avio_wb32(pb, rtpStats_.jitter >> 4); /* jitter */

  if (rtpStats_.last_rtcp_ntp_time == AV_NOPTS_VALUE) {
    avio_wb32(pb, 0); /* last SR timestamp */
    avio_wb32(pb, 0); /* delay since last SR */
  } else {
    uint32_t middle_32_bits = rtpStats_.last_rtcp_ntp_time >>
                              16;  // this is valid, right? do we need to
                                   // handle 64 bit values special?
    uint32_t delay_since_last =
        av_rescale(av_gettime_relative() - rtpStats_.last_rtcp_reception_time,
                   65536, AV_TIME_BASE);

    avio_wb32(pb, middle_32_bits);   /* last SR timestamp */
    avio_wb32(pb, delay_since_last); /* delay since last SR */
  }

  // CNAME
  avio_w8(pb, (RTP_VERSION << 6) + 1); /* 1 report block */
  avio_w8(pb, RTCP_SDES);
  int len = strlen(rtpStats_.hostname);
  avio_wb16(pb, (7 + len + 3) / 4); /* length in words - 1 */
  avio_wb32(pb, rtpStats_.ssrc + 1);
  avio_w8(pb, 0x01);
  avio_w8(pb, len);
  avio_write(pb, rtpStats_.hostname, len);
  avio_w8(pb, 0); /* END */
  // padding
  for (len = (7 + len) % 4; len % 4; len++) avio_w8(pb, 0);
#endif
}

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

    int ret =
        this->belongingSSRCInfo_.belongingRtpHandler.belongingPeerConnection_
            .rtcpHandler_.rtpfb_.nack_.CreateNackSend(nackSeqs, kSelfRtcpSSRC,
                                                      kMediaSrcSSRC);
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

  int ret = belongingSSRCInfo_.belongingRtpHandler.belongingPeerConnection_
                .rtcpHandler_.psfb_.pli_.CreatePLISend();
  if (ret) {
    tylog("createPLIReportSend ret=%d", ret);
    // not return
  }

  // OPT: discard P frame, reserve and pop I frame
  jitterBuffer_.clear();
  lastPoppedPowerSeq_ = kShitRecvPowerSeqInitValue;

  return {};
}

}  // namespace tywebrtc
