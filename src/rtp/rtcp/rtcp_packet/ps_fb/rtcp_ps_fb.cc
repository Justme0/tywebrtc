// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/ps_fb/rtcp_ps_fb.h"

#include <cassert>
#include <cinttypes>

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"
#include "src/signal/signal_handler.h"

namespace tywebrtc {

RtcpPayloadSpecificFeedback::RtcpPayloadSpecificFeedback(
    RtcpHandler& belongingRtcpHandler)
    : belongingRtcpHandler_(belongingRtcpHandler), pli_(*this) {}

int RtcpPayloadSpecificFeedback::HandlePayloadSpecificFeedback(
    const RtcpHeader& chead) {
  int ret = 0;

  tylog("specific feedback recv type [%s]",
        RtcpPayloadSpecificFormatToString(
            static_cast<RtcpPayloadSpecificFormat>(chead.getBlockCount()))
            .data());

  switch (static_cast<RtcpPayloadSpecificFormat>(chead.getBlockCount())) {
    case RtcpPayloadSpecificFormat::kRtcpPLI: {
      ret = this->pli_.HandlePLI(chead);
      if (ret) {
        tylog("handle pli ret=%d", ret);

        return ret;
      }

      break;
    }

    case RtcpPayloadSpecificFormat::kRtcpSLI:
      break;

    case RtcpPayloadSpecificFormat::kRtcpFIR:
      break;

    case RtcpPayloadSpecificFormat::kRtcpRPSI:
      break;

    case RtcpPayloadSpecificFormat::kRtcpREMB:
      break;

    default:
      // should only log
      assert(!"unknown rtcp");
      break;
  }

  return 0;
}

// int RtcpPayloadSpecificFeedback::CreatePayloadSpecificFeedback(
//     RtcpPayloadSpecificFormat fmt, std::vector<char>* io_rtcpBin) {
//   return 0;
// }

}  // namespace tywebrtc
