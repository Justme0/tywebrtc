// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/payload_spec_fb/rtcp_pli.h"

#include <cassert>

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"
#include "src/rtp/rtcp/rtcp_handler.h"

namespace tywebrtc {

RtcpPLI::RtcpPLI(RtcpHandler &belongingRtcpHandler)
    : belongingRtcpHandler_(belongingRtcpHandler) {}

int RtcpPLI::CreatePLISend(uint32_t ssrc, uint32_t sourceSSRC) {
  int ret = 0;

  tylog("create PLI, localSSRC=%u, remoteSSRC=%u.", ssrc, sourceSSRC);

  RtcpHeader pli;
  pli.setPacketType(RtcpPacketType::kPayloadSpecificFeedback);
  pli.setBlockCount(static_cast<uint8_t>(RtcpPayloadSpecificFormat::kRtcpPLI));
  pli.setSSRC(ssrc);
  pli.setMediaSourceSSRC(sourceSSRC);
  pli.setLength(2);

  const char *head = reinterpret_cast<const char *>(&pli);
  const int len = (pli.getLength() + 1) * 4;
  assert(len == 12);                            // head len
  std::vector<char> rtcpBin(head, head + len);  // can use string view

  DumpSendPacket(rtcpBin);

  ret = this->belongingRtcpHandler_.belongingPeerConnection_.srtpHandler_
            .ProtectRtcp(const_cast<std::vector<char> *>(&rtcpBin));
  if (ret) {
    tylog("uplink send to src client, protect rtcp ret=%d", ret);

    return ret;
  }

  ret = this->belongingRtcpHandler_.belongingPeerConnection_.SendToClient(
      rtcpBin);
  if (ret) {
    tylog("send to client nack rtcp ret=%d", ret);

    return ret;
  }

  tylog("send PLI succ, ssrc=%u, source ssrc=%u.", ssrc, sourceSSRC);

  return 0;
}

}  // namespace tywebrtc