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
  // rtcp head is 4 bytes, sender ssrc is 4 bytes
  int blockBufSize = chead.getRealLength() - 8;
  while (true) {
    const RtcpHeader& blockHead =
        *reinterpret_cast<const RtcpHeader*>(blockMovPointer);

    const uint32_t blockLen = blockHead.getBlockLen();
    tylog("[ExtendedReport] BlockLen[%u]", blockLen);

    // block head is 4 bytes.
    // https://datatracker.ietf.org/doc/html/rfc3611#section-3
    // If the block type definition permits,
    // zero is an acceptable value, signifying a block that consists
    // of only the BT, type-specific, and block length fields, with a
    // null type-specific block contents field.
    if (blockLen == 0 || blockHead.getBlockRealLen() > blockBufSize) {
      tylog("[ExtendedReport]BlockLen[%u]*4 + 4 > blockBufSize[%u]", blockLen,
            blockBufSize);
      break;
    }
    blockBufSize -= blockHead.getBlockRealLen();
    assert(blockBufSize >= 0);

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

    blockMovPointer += blockHead.getBlockRealLen();
  }

  return 0;
}

}  // namespace tywebrtc
