// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/ps_fb/rtcp_pli.h"

#include <cassert>

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"
#include "src/rtp/rtcp/rtcp_handler.h"

namespace tywebrtc {

RtcpPLI::RtcpPLI(RtcpPayloadSpecificFeedback &belongingPsfb)
    : belongingPsfb_(belongingPsfb) {}

// handle downlink
int RtcpPLI::HandlePLI(const RtcpHeader &) {
  int ret = 0;

  auto peerPC =
      this->belongingPsfb_.belongingRtcpHandler_.belongingPC_.FindPeerPC();
  if (nullptr == peerPC) {
    tylog(
        "another peerPC null(may only pull rtmp/srt/...), can not "
        "req I frame.");

    return 0;
  }

  if (peerPC->rtpHandler_.upVideoSSRC == 0) {
    tylog(
        "peerPC upVideo SSRC=0(maybe in begin time), so not transfer "
        "video req to it. peerPC=%s.",
        peerPC->ToString().data());

    return 0;
  }

  // req peer uplink
  ret = peerPC->rtcpHandler_.psfb_.pli_.CreatePLISend();
  if (ret) {
    tylog("create pli ret=%d", ret);
    return ret;
  }

  return 0;
}

int RtcpPLI::CreatePLISend() {
  int ret = 0;

  const uint32_t sourceSSRC = this->belongingPsfb_.belongingRtcpHandler_
                                  .belongingPC_.rtpHandler_.upVideoSSRC;

  tylog("create PLI, localSSRC=%u, remoteSSRC=%u.", kSelfRtcpSSRC, sourceSSRC);

  RtcpHeader pli;
  pli.setPacketType(RtcpPacketType::kPayloadSpecificFeedback);
  pli.setBlockCount(static_cast<uint8_t>(RtcpPayloadSpecificFormat::kRtcpPLI));
  pli.setSSRC(kSelfRtcpSSRC);
  pli.setSourceSSRC(sourceSSRC);
  pli.setLength(2);

  const char *head = reinterpret_cast<const char *>(&pli);
  const int len = pli.getRealLength();
  assert(len == 12);                            // head len
  std::vector<char> rtcpBin(head, head + len);  // can use string view

  DumpSendPacket(rtcpBin);

  ret = this->belongingPsfb_.belongingRtcpHandler_.belongingPC_.srtpHandler_
            .ProtectRtcp(const_cast<std::vector<char> *>(&rtcpBin));
  if (ret) {
    tylog("protect rtcp ret=%d", ret);

    return ret;
  }

  ret = this->belongingPsfb_.belongingRtcpHandler_.belongingPC_.SendToClient(
      rtcpBin);
  if (ret) {
    tylog("send to client ret=%d", ret);

    return ret;
  }

  tylog("send PLI succ, ssrc=%u, source ssrc=%u.", kSelfRtcpSSRC, sourceSSRC);

  return 0;
}

}  // namespace tywebrtc