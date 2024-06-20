// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_receiver_report.h"

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

RtcpReceiverReport::RtcpReceiverReport(RtcpHandler& belongingRtcpHandler)
    : belongingRtcpHandler_(belongingRtcpHandler) {}

// ref
// https://source.chromium.org/chromium/chromium/src/+/main:third_party/webrtc/modules/rtp_rtcp/source/rtcp_receiver.cc;l=576
int RtcpReceiverReport::HandleReceiverReport(const RtcpHeader& chead) {
  uint8_t block_count = chead.getBlockCount();
  tylog("recv RR block_count=%d.", block_count);
  const uint16_t rr_length = chead.getRealLength();
  const uint32_t ssrc = chead.getSSRC();

  // RR report block is 24 B
  // https://datatracker.ietf.org/doc/html/rfc3550#section-6.4.2
  // |                 SSRC_1 (SSRC of first source)                 |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // | fraction lost |       cumulative number of packets lost       |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |           extended highest sequence number received           |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                      interarrival jitter                      |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                         last SR (LSR)                         |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                   delay since last SR (DLSR)                  |
  constexpr const int kRRReportBlockSize =
      sizeof(RtcpHeader::report_t::receiverReport_t);
  static_assert(24 == kRRReportBlockSize,
                "RRReportBlockSize size should be 24");

  const int kRTCPHeadSize = 8;
  if (kRTCPHeadSize + block_count * kRRReportBlockSize != rr_length) {
    tylog("[ReceiverReport] ssrc[%u] BlockCount[%u] rrLen[%u], invalid RR!",
          ssrc, block_count, rr_length);
    return -1;
  }

  // hack: pDummyHead points to `last SR (LSR)` of last block,
  // cannot get RTCP head field.
  int i;
  const RtcpHeader* pDummyHead;
  for (i = 0, pDummyHead = &chead; i < block_count; ++i,
      pDummyHead = reinterpret_cast<const RtcpHeader*>(
          reinterpret_cast<const char*>(pDummyHead) + kRRReportBlockSize)) {
    uint32_t sourceSsrc = pDummyHead->getSourceSSRC();
    // Fraction Lost
    uint8_t fractLost = pDummyHead->getFractionLost();
    // cumulative number of packets lost
    uint32_t lostPkgs = pDummyHead->getLostPackets();
    uint16_t seqnumCycles = pDummyHead->getSeqnumCycles();
    uint16_t highestSeq = pDummyHead->getHighestSeqnum();
    uint32_t jitter = pDummyHead->getJitter();

    const uint32_t kLSR = pDummyHead->getLastSr();
    const uint32_t kDLSR = pDummyHead->getDelaySinceLastSr();

    uint32_t rttNtp = 0;
    int rttMs = 0;
    if (kLSR > 0 && kDLSR > 0) {
      rttNtp = CompactNtp(MsToNtp(g_now_ms)) - kDLSR - kLSR;
      rttMs = CompactNtpRttToMs(rttNtp);
      // SaveDownLost(sourceSsrc, fractLost, lostPkgs, rtt);
      this->belongingRtcpHandler_.belongingPC_.signalHandler_.S2CReportRTT(
          rttMs);
    }

    tylog(
        "RRSrcSsrc=%u,lostRate[%u,%d%%]allLost=%u,cycle=%u,hiSeq=%u,jit=%u,LSR="
        "%u,DLSR[%u=>%" PRIi64 "ms],rttNtp[%u=>%dms]",
        sourceSsrc, fractLost, fractLost * 100 / 256, lostPkgs, seqnumCycles,
        highestSeq, jitter, kLSR, kDLSR,
        kDLSR == 0 ? 0 : CompactNtpRttToMs(kDLSR), rttNtp, rttMs);

    auto& m = belongingRtcpHandler_.belongingPC_.rtpHandler_.ssrcInfoMap_;
    auto it = m.find(sourceSsrc);
    if (it == m.end()) {
      tylog("rr source_ssrc=%u not in ssrc info map=%s.", sourceSsrc,
            tylib::AnyToString(m).data());

      // should continue?
      return 0;
    }

    RrPkgInfo* info = &it->second.rrInfo_;
    info->recvMs = g_now_ms;
    info->RRCount++;
    info->fractionLost = fractLost;
  }

  return 0;
}

int RtcpReceiverReport::CreateReceiverReport(const RtpReceiver& receiver,
                                             std::vector<char>* io_rtcpBin) {
  RTPStatistics& rtpStats = receiver.rtpStats_;

  rtpStats.last_octet_count = rtpStats.octet_count;
  // some placeholders we should really fill...
  // RFC 1889/p64
  uint32_t extended_max = rtpStats.cycles + rtpStats.max_seq;
  uint32_t expected = extended_max - rtpStats.base_seq;
  uint32_t lost = expected - rtpStats.received;
  // clamp it since it's only 24 bits...
  lost = std::min<uint32_t>(lost, 0xffffff);
  uint32_t expected_interval = expected - rtpStats.expected_prior;
  rtpStats.expected_prior = expected;
  uint32_t received_interval = rtpStats.received - rtpStats.received_prior;
  rtpStats.received_prior = rtpStats.received;
  int32_t lost_interval = expected_interval - received_interval;

  uint8_t fraction = 0;
  if (expected_interval == 0 || lost_interval <= 0) {
    fraction = 0;
  } else {
    fraction = (lost_interval << 8) / expected_interval;
  }

  const int uiLocSsrc = 23333;

  assert(g_now_ms >= receiver.belongingSSRCInfo_.srInfo_.recvMs);

  RtcpHeader receiverReport;
  receiverReport.setBlockCount(1);
  receiverReport.setPacketType(RtcpPacketType::kReceiverReport);
  receiverReport.setLength(7);  // OPT: magic number
  receiverReport.setSSRC(uiLocSsrc);

  receiverReport.setSourceSSRC(receiver.belongingSSRCInfo_.ssrc_key_);
  receiverReport.setFractionLost(fraction);
  receiverReport.setLostPackets(lost);
  receiverReport.setSeqnumCycles(rtpStats.cycles);
  receiverReport.setHighestSeqnum(rtpStats.max_seq);
  // Note: internal jitter value is in Q4 and needs to be scaled by 1/16.
  receiverReport.setJitter(rtpStats.jitter_q4_ >> 4);
  receiverReport.setLastSr(
      CompactNtp(NtpTime(receiver.belongingSSRCInfo_.srInfo_.NTPTimeStamps)));
  receiverReport.setDelaySinceLastSr(
      CompactNtp(MsToNtp(g_now_ms)) -
      CompactNtp(MsToNtp(receiver.belongingSSRCInfo_.srInfo_.recvMs)));

  tylog("create rr=%s.", receiverReport.ToString().data());

  const char* buf = reinterpret_cast<const char*>(&receiverReport);
  io_rtcpBin->insert(io_rtcpBin->end(), buf,
                     buf + receiverReport.getRealLength());

  return 0;
}

}  // namespace tywebrtc