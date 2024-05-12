// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_xr/block_rrtr.h"

#include <cinttypes>

#include "src/global_tmp/global_tmp.h"
#include "src/log/log.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

RtcpRRTR::RtcpRRTR(RtcpExtendedReports& belongingXr)
    : belongingXr_(belongingXr) {
  (void)belongingXr_;
}

int RtcpRRTR::HandleRtcpRRTR(const RtcpHeader& blockHead) {
  uint32_t rrtrSsrc = blockHead.getSSRC();
  uint64_t rrtrNtp = blockHead.getRrtrNtp();
  uint64_t netNtp = *((uint64_t*)((char*)&blockHead + 12));

  tylog("[RRTR] rrtrSsrc:%u, rrtrNtp:%" PRIu64 ", ntp:%" PRIu64 ".", rrtrSsrc,
        rrtrNtp, be64toh(netNtp));

  // auto lastRrtrNtp = CompactNtp(NtpTime(rrtrNtp));
  // auto rcvRrtrTime = g_now_ms;
  // auto rrtrSsrc = rrtrSsrc;

  return 0;
}

int RtcpRRTR::CreateRtcpRRTR(std::vector<char>* io_rtcpBin) {
  RtcpHeader rrtrReport;
  // no need block count ?
  rrtrReport.setPacketType(RtcpPacketType::kExtendedReports);
  rrtrReport.setLength(4);
  rrtrReport.setSSRC(this->belongingXr_.belongingRtcpHandler_
                         .belongingPeerConnection_.rtpHandler_.upVideoSSRC);

  rrtrReport.setBlockType(EnXRBlockType::kXRBlockRRTR);
  const int kRRTRBlockLen = 2;  // all block is (2+1)*4 B
  rrtrReport.setBlockLen(kRRTRBlockLen);
  // KEY: peer SR will return this ntp time
  rrtrReport.setRrtrNtp(MsToNtp(g_now_ms).GetValue());

  char* buf = reinterpret_cast<char*>(&rrtrReport);
  int len = (rrtrReport.getLength() + 1) * 4;

  io_rtcpBin->insert(io_rtcpBin->end(), buf, buf + len);

  return 0;
}

}  // namespace tywebrtc
