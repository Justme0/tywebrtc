// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// RTCP XR RFC: https://datatracker.ietf.org/doc/html/rfc3611

#ifndef SRC_RTP_RTCP_RTCP_PACKET_PS_FB_RTCP_PS_FB_H_
#define SRC_RTP_RTCP_RTCP_PACKET_PS_FB_RTCP_PS_FB_H_

#include "src/rtp/rtcp/rtcp_packet/ps_fb/rtcp_pli.h"
#include "src/rtp/rtcp/rtcp_packet/ps_fb/rtcp_remb.h"

#include <vector>

namespace tywebrtc {

class RtcpHandler;
class RtcpHeader;

// PSFB: Payload-specific feedback message.
// https://datatracker.ietf.org/doc/html/rfc4585#section-6.3
class RtcpPayloadSpecificFeedback {
 public:
  explicit RtcpPayloadSpecificFeedback(RtcpHandler &belongingRtcpHandler);

  int HandlePayloadSpecificFeedback(const RtcpHeader &chead);
  // int CreatePayloadSpecificFeedback(RtcpPayloadSpecificFormat fmt,
  //                                   std::vector<char> *io_rtcpBin);

  RtcpHandler &belongingRtcpHandler_;

  RtcpPLI pli_;
  RtcpREMB remb_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PACKET_PS_FB_RTCP_PS_FB_H_