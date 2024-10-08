// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// RFC: https://datatracker.ietf.org/doc/html/draft-alvestrand-rmcat-remb-03

#include "src/rtp/rtcp/rtcp_packet/ps_fb/rtcp_remb.h"

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"
#include "src/rtp/rtcp/rtcp_handler.h"

namespace tywebrtc {

RtcpREMB::RtcpREMB(RtcpPayloadSpecificFeedback &belongingPsfb)
    : belongingPsfb_(belongingPsfb) {}

// handle downlink
int RtcpREMB::HandleREMB(const RtcpHeader &) {
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

  (void)ret;

  return 0;
}

int RtcpREMB::CreateREMB(const std::vector<uint32_t> &ssrcApplied,
                         std::vector<char> *io_rtcpBin) {
  RtcpHeader remb;
  remb.setPacketType(RtcpPacketType::kPayloadSpecificFeedback);
  remb.setBlockCount(
      static_cast<uint8_t>(RtcpPayloadSpecificFormat::kRtcpREMB));
  memcpy(&remb.report.rembPacket.uniqueid, "REMB", 4);
  remb.setSSRC(kSelfRtcpSSRC);
  // SSRC of media source (32 bits):  Always 0; this is the same convention as
  // in [RFC5104] section 4.2.2.2 (TMMBN).
  remb.setSourceSSRC(0);
  remb.setLength(5);
  remb.setREMBBitRate(233222);
  remb.setREMBNumSSRC(ssrcApplied.size());
  for (size_t i = 0; i != ssrcApplied.size(); ++i) {
    remb.setREMBFeedSSRC(i, ssrcApplied[i]);
  }
  tylog("create REMB=%s.", remb.ToString().data());

  const char *buf = reinterpret_cast<const char *>(&remb);
  io_rtcpBin->insert(io_rtcpBin->end(), buf, buf + remb.getRealLength());

  return 0;
}

}  // namespace tywebrtc