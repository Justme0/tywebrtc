// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_extended_reports.h"

namespace tywebrtc {

RtcpExtendedReports::RtcpExtendedReports(RtcpHandler &belongingRtcpHandler)
    : belongingRtcpHandler_(belongingRtcpHandler) {}

int RtcpExtendedReports::HandleExtendedReports(const RtcpHeader &chead) {
  (void)chead;
  return 0;
}

}  // namespace tywebrtc