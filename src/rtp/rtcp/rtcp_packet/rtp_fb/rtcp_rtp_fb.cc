// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtp_fb/rtcp_rtp_fb.h"

#include <cassert>
#include <cinttypes>

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

RtcpRtpFeedback::RtcpRtpFeedback(RtcpHandler& belongingRtcpHandler)
    : belongingRtcpHandler_(belongingRtcpHandler), nack_(*this) {}

int RtcpRtpFeedback::HandleRtpFeedback(const RtcpHeader& chead) {
  int ret = 0;

  switch (static_cast<RtcpRtpFeedbackFormat>(chead.getBlockCount())) {
    case RtcpRtpFeedbackFormat::kFeedbackNack: {
      ret = this->nack_.HandleNack(chead);
      if (ret) {
        tylog("handleNack ret=%d", ret);

        return ret;
      }

      break;
    }

    case RtcpRtpFeedbackFormat::kFeedbackTCC: {
      tylog("TCC feedback rtcp pkt.");

      break;
    }

    default:
      tylog("unknown rtp fb=%d.", chead.getBlockCount());
      assert(!"unknown rtcp");
      break;
  }
  return 0;
}

/*
int RtcpRtpFeedback::CreateRtpFeedback(RtcpRtpFeedbackFormat fmt,
                                       std::vector<char>* io_rtcpBin) {
  (void)fmt;
  (void)io_rtcpBin;
  return 0;
}
*/

}  // namespace tywebrtc
