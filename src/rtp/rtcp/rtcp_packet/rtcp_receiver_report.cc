// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_receiver_report.h"

#include "src/global_tmp/global_tmp.h"
#include "src/rtp/rtcp/rtcp_handler.h"
#include "src/rtp/rtcp/rtcp_parser.h"

namespace tywebrtc {

RtcpReceiverReport::RtcpReceiverReport(RtcpHandler &belongingRtcpHandler)
    : belongingRtcpHandler_(belongingRtcpHandler) {}

int RtcpReceiverReport::HandleReceiverReport(const RtcpHeader &chead) {
  uint8_t block_count = chead.getBlockCount();
  tylog("recv RR block_count=%d.", block_count);
  uint16_t rr_length = (chead.getLength() + 1) * 4;
  uint32_t ssrc = chead.getSSRC();

  if (8 + block_count * 24 != rr_length) {
    tylog("[ReceiverReport] ssrc[%u] BlockCount[%u] rrLen[%u], invalid RR!",
          ssrc, block_count, rr_length);
    return -1;
  }

  // uint8_t *pos = reinterpret_cast<uint8_t *>(chead);

  for (int i = 0; i < block_count; i++) {
    // uint32_t sourceSsrc = chead->getSourceSSRC();
    // uint8_t fractLost = chead->getFractionLost();  // Fraction Lost
    // uint32_t lostPkgs =
    //     chead->getLostPackets();  // cumulative number of packets lost
    // uint16_t seqnumCycles = chead->getSeqnumCycles();
    // uint16_t highestSeq = chead->getHighestSeqnum();
    // uint32_t jitter = chead->getJitter();
    uint32_t lastSr = chead.getLastSr();
    tylog("last sr=%u", lastSr);
    assert(0 != lastSr);  // should not use assert :)
    uint32_t sinceDelay = chead.getDelaySinceLastSr();

    NtpTime nowNtp = MsToNtp(g_now_ms);
    // key:
    uint32_t rttNtp = CompactNtp(nowNtp) - sinceDelay - lastSr;
    int rttMs = CompactNtpRttToMs(rttNtp);
    tylog("rtt ms=%d.", rttMs);
  }

  return 0;
}

}  // namespace tywebrtc