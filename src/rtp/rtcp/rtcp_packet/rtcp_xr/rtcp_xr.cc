// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_xr/rtcp_xr.h"

#include <cassert>
#include <cinttypes>

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"
#include "src/signal/signal_handler.h"

namespace tywebrtc {

RtcpExtendedReports::RtcpExtendedReports(RtcpHandler& belongingRtcpHandler)
    : belongingRtcpHandler_(belongingRtcpHandler), rrtr_(*this), dlrr_(*this) {}

int RtcpExtendedReports::HandleExtendedReports(const RtcpHeader& chead) {
  const uint8_t* blockMovPointer = reinterpret_cast<const uint8_t*>(&chead);
  uint32_t rtcpLen = (chead.getLength() + 1) * 4;
  // rtcp head is 4 bytes, sender ssrc is 4 bytes
  uint32_t blockBufSize = rtcpLen - 8;
  while (true) {
    const RtcpHeader& blockHead =
        *reinterpret_cast<const RtcpHeader*>(blockMovPointer);

    uint32_t blockLen = blockHead.getBlockLen();
    tylog("[ExtendedReport] blockLen[%u]", blockLen);

    // block head is 4 bytes
    if (blockLen == 0 || (blockLen * 4 + 4) > blockBufSize) {
      tylog("[ExtendedReport]blockLen[%u]*4 + 4 > blockBufSize[%u]", blockLen,
            blockBufSize);
      break;
    }
    blockBufSize = blockBufSize - blockLen * 4 - 4;

    switch (blockHead.getBlockType()) {
      // ref
      // https://source.chromium.org/chromium/chromium/src/+/main:third_party/webrtc/modules/rtp_rtcp/source/rtcp_receiver.cc;l=851
      case EnXRBlockType::kXRBlockDLRR: {
        dlrr_.HandleRtcpDLRR(blockHead);

        break;
      }

      case EnXRBlockType::kXRBlockRRTR: {
        rrtr_.HandleRtcpRRTR(blockHead);

        break;
      }

      default:
        tylog("not support block type=%d.", blockHead.getBlockType());
        assert(!"not support :)");
    }

    if (blockBufSize <= 0) break;

    blockMovPointer += (blockLen + 1) * 4;
  }

  return 0;
}

}  // namespace tywebrtc