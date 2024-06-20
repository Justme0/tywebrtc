// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_xr/block_rrtr.h"

#include <cinttypes>

#include "src/global_tmp/global_tmp.h"
#include "src/log/log.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

RtcpRRTR::RtcpRRTR(RtcpExtendedReports& belongingXr)
    : belongingXr_(belongingXr) {}

int RtcpRRTR::HandleRtcpRRTR(const RtcpHeader& blockHead) {
  // no use? or should set in DLRR?
  uint32_t rrtrSsrc = blockHead.getSSRC();
  uint64_t rrtrNtp = blockHead.getRrtrNtp();

  tylog("[RRTR] rrtrSsrc:%u, rrtrNtp:%" PRIu64 ".", rrtrSsrc, rrtrNtp);

  assert(rrtrNtp != 0 &&
         "A report sender that has no notion of wallclock or elapsed time may "
         "set the NTP timestamp to zero.");

  RrtrPkgInfo* info = &this->belongingXr_.belongingRtcpHandler_.belongingPC_
                           .rtpHandler_.rrtrInfo_;
  info->recvMs = g_now_ms;
  info->rrtrNtp = rrtrNtp;

  return 0;
}

int RtcpRRTR::CreateRtcpRRTR(uint32_t ssrc, std::vector<char>* io_rtcpBin) {
  RtcpHeader rrtrReport;
  // no need block count ?
  rrtrReport.setPacketType(RtcpPacketType::kExtendedReports);
  rrtrReport.setLength(4);
  rrtrReport.setSSRC(ssrc);

  assert(ssrc ==
             belongingXr_.belongingRtcpHandler_.belongingPC_.rtpHandler_
                 .upAudioSSRC ||
         ssrc ==
             belongingXr_.belongingRtcpHandler_.belongingPC_.rtpHandler_
                 .upVideoSSRC);

  rrtrReport.setBlockType(EnXRBlockType::kXRBlockRRTR);
  const int kRRTRBlockLen = 2;  // all block is (2+1)*4 B
  rrtrReport.setBlockLen(kRRTRBlockLen);
  // KEY: used for peer SR which will return this ntp time
  rrtrReport.setRrtrNtp(MsToNtp(g_now_ms).GetValue());

  char* buf = reinterpret_cast<char*>(&rrtrReport);
  io_rtcpBin->insert(io_rtcpBin->end(), buf, buf + rrtrReport.getRealLength());

  return 0;
}

}  // namespace tywebrtc
