// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// RTCP XR RFC: https://datatracker.ietf.org/doc/html/rfc3611

#ifndef SRC_RTP_RTCP_RTCP_PACKET_RTCP_XR_RTCP_XR_H_
#define SRC_RTP_RTCP_RTCP_PACKET_RTCP_XR_RTCP_XR_H_

#include "src/rtp/rtcp/rtcp_packet/rtcp_xr/block_dlrr.h"
#include "src/rtp/rtcp/rtcp_packet/rtcp_xr/block_rrtr.h"
#include "src/rtp/rtcp/rtcp_parser.h"

namespace tywebrtc {

class RtcpHandler;
class RtcpHeader;

class RtcpExtendedReports {
 public:
  explicit RtcpExtendedReports(RtcpHandler &belongingRtcpHandler);

  int HandleExtendedReports(const RtcpHeader &chead);
  int CreateExtendedReportsSend();

  RtcpHandler &belongingRtcpHandler_;

 private:
  RtcpRRTR rrtr_;
  RtcpDLRR dlrr_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PACKET_RTCP_XR_RTCP_XR_H_