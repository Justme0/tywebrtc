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

const uint32_t kBlockHeadBytes = 4;  // 1 type + 1 typeSpecific + 2 len
const uint32_t kSubBlockBytes = 12;  // 4 ssrc + 4 lasr rr + 4 delay rr

RtcpDLRR::RtcpDLRR(RtcpExtendedReports& belongingXr)
    : belongingXr_(belongingXr) {}

int RtcpDLRR::HandleRtcpDLRR(const RtcpHeader& blockHead) {
  // OPT: should check bound
  const uint32_t onlyBlocksSize = blockHead.getBlockRealLen() - kBlockHeadBytes;
  assert(onlyBlocksSize % kSubBlockBytes == 0);
  const int blockCount = onlyBlocksSize / kSubBlockBytes;
  tylog("blockcount=%d.", blockCount);

  for (const char *worker = reinterpret_cast<const char *>(&blockHead),
                  *const end = worker + onlyBlocksSize;
       worker != end; worker += kSubBlockBytes) {
    // pdlrrHead is not real RtcpHeader, can only get SSRC sub-block field
    const RtcpHeader* pdlrrHead = reinterpret_cast<const RtcpHeader*>(worker);

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

      this->belongingXr_.belongingRtcpHandler_.belongingPC_.signalHandler_
          .S2CReportRTT(rttMs);
    }
  }

  return 0;
}

int RtcpDLRR::CreateRtcpDLRR(const RtpSender& sender,
                             std::vector<char>* io_rtcpBin) {
  if (sender.belongingSSRCInfo_.belongingRtpHandler.rrtrInfo_.recvMs == 0 ||
      sender.belongingSSRCInfo_.belongingRtpHandler.rrtrInfo_.rrtrNtp == 0) {
    tylog("not ever recv RRTR, so not send DLRR");

    return 0;
  }

  RtcpHeader dlrr;
  dlrr.setPacketType(RtcpPacketType::kExtendedReports);
  dlrr.setSSRC(sender.belongingSSRCInfo_.ssrc_key_);
  dlrr.setLength(5);  // one block
  dlrr.setBlockCount(1);

  dlrr.setBlockType(EnXRBlockType::kXRBlockDLRR);
  dlrr.setBlockLen(3);  // one block
  dlrr.setRrSsrc(sender.belongingSSRCInfo_.ssrc_key_);
  dlrr.setLastRr(CompactNtp(NtpTime(
      sender.belongingSSRCInfo_.belongingRtpHandler.rrtrInfo_.rrtrNtp)));
  dlrr.setDLRR(
      CompactNtp(MsToNtp(g_now_ms)) -
      CompactNtp(MsToNtp(
          sender.belongingSSRCInfo_.belongingRtpHandler.rrtrInfo_.recvMs)));

  char* buf = reinterpret_cast<char*>(&dlrr);
  io_rtcpBin->insert(io_rtcpBin->end(), buf, buf + dlrr.getRealLength());

  return 0;
}

}  // namespace tywebrtc
