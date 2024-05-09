// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_xr/block_rrtr.h"

#include <cinttypes>

#include "src/log/log.h"

namespace tywebrtc {

RtcpRRTR::RtcpRRTR(RtcpExtendedReports& belongingXrHandler)
    : belongingXrHandler_(belongingXrHandler) {
  (void)belongingXrHandler_;
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

}  // namespace tywebrtc