// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_RTCP_RTCP_PACKET_PS_FB_RTCP_PLI_H_
#define SRC_RTP_RTCP_RTCP_PACKET_PS_FB_RTCP_PLI_H_

#include <cstdint>

#include "src/rtp/rtcp/rtcp_parser.h"

namespace tywebrtc {

class RtcpPayloadSpecificFeedback;

class RtcpPLI {
 public:
  explicit RtcpPLI(RtcpPayloadSpecificFeedback &belongingPsfb);
  int HandlePLI(const RtcpHeader &chead);

  int CreatePLISend();

 private:
  RtcpPayloadSpecificFeedback &belongingPsfb_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PACKET_PS_FB_RTCP_PLI_H_