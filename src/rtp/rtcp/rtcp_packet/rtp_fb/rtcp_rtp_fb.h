// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// RTCP XR RFC: https://datatracker.ietf.org/doc/html/rfc3611

#ifndef SRC_RTP_RTCP_RTCP_PACKET_RTP_FB_RTCP_RTP_FB_H_
#define SRC_RTP_RTCP_RTCP_PACKET_RTP_FB_RTCP_RTP_FB_H_

#include "src/rtp/rtcp/rtcp_packet/rtp_fb/rtcp_nack.h"
#include "src/rtp/rtcp/rtcp_parser.h"

namespace tywebrtc {

class RtcpHandler;

// RTPFB: Transport layer feedback message.
// https://datatracker.ietf.org/doc/html/rfc4585#section-6.2
class RtcpRtpFeedback {
 public:
  explicit RtcpRtpFeedback(RtcpHandler &belongingRtcpHandler);

  int HandleRtpFeedback(const RtcpHeader &chead);
  // int CreateRtpFeedback(EnXRBlockType blockType, std::vector<char>
  // *io_rtcpBin);

  RtcpHandler &belongingRtcpHandler_;

  RtcpNack nack_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PACKET_RTP_FB_RTCP_RTP_FB_H_