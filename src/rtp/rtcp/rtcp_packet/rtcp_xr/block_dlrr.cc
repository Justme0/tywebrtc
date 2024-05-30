// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_xr/block_dlrr.h"

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

RtcpDLRR::RtcpDLRR(RtcpExtendedReports& belongingXr)
    : belongingXr_(belongingXr) {}

int RtcpDLRR::HandleRtcpDLRR(const RtcpHeader& blockHead) {
  uint32_t blockLen = blockHead.getBlockLen();
  uint32_t subBlockBytes = 12;  // 4 ssrc + 4 lasr rr + 4 delay rr,
  if ((blockLen * 4) % subBlockBytes != 0) {
    tylog("[ExtendedReport] ((blockLen[%u])*4) %% 12 !=  0", blockLen);
    return -1;
  }

  uint32_t blockCount = (blockLen)*4 / subBlockBytes;
  const uint8_t* dlrrMovPointer = reinterpret_cast<const uint8_t*>(&blockHead);
  const RtcpHeader* pdlrrHead;
  if (blockCount > 1) {
    tylog("blockcount=%d.", blockCount);
  }
  for (uint32_t i = 0; i < blockCount; i++) {
    pdlrrHead = reinterpret_cast<const RtcpHeader*>(dlrrMovPointer);

    uint32_t rrSsrc = pdlrrHead->getRrSsrc();

    uint32_t lastRr = pdlrrHead->getLastRr();
    uint32_t dlrr = pdlrrHead->getDLRR();
    if (lastRr > 0 && dlrr > 0) {
      uint32_t rttNtp = CompactNtp(MsToNtp(g_now_ms)) - dlrr - lastRr;
      uint32_t rttMs = CompactNtpRttToMs(rttNtp);

      // rtt_acumulator_.Update(g_now_ms, rttMs);
      tylog("taylor [ExtendedReport] rrSsrc[%u] lastRr[%u] dlrr[%u=>%" PRIi64
            "ms] rttNtp[%u=>%" PRIu32 "ms]",
            rrSsrc, lastRr, dlrr, CompactNtpRttToMs(dlrr), rttNtp, rttMs);

      this->belongingXr_.belongingRtcpHandler_.belongingPeerConnection_
          .signalHandler_.S2CReportRTT(rttMs);
    }

    break;
  }

  return 0;
}

int RtcpDLRR::CreateRtcpDLRR(std::vector<char>* io_rtcpBin) {
  // taylor fixme
  uint32_t dlrrNtp = CompactNtp(NtpTime(1, 0));

  // mock
  const int rrtr_ssrc = 1;   // ?
  uint32_t lastRrtrNtp = 3;  // test

  RtcpHeader dlrr;
  dlrr.setPacketType(RtcpPacketType::kExtendedReports);
  dlrr.setSSRC(kDownlinkVideoSsrc);
  dlrr.setLength(5);
  dlrr.setBlockCount(1);

  dlrr.setBlockType(EnXRBlockType::kXRBlockDLRR);
  dlrr.setBlockLen(3);
  dlrr.setRrSsrc(rrtr_ssrc);
  dlrr.setLastRr(lastRrtrNtp);
  dlrr.setDLRR(dlrrNtp);

  char* buf = reinterpret_cast<char*>(&dlrr);
  int len = (dlrr.getLength() + 1) * 4;

  io_rtcpBin->insert(io_rtcpBin->end(), buf, buf + len);

  return 0;
}

}  // namespace tywebrtc
