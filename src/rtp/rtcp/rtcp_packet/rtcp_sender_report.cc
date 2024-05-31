// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_sender_report.h"

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"
#include "src/rtp/rtcp/rtcp_parser.h"

namespace tywebrtc {

RtcpSenderReport::RtcpSenderReport(RtcpHandler& belongingRtcpHandler)
    : belongingRtcpHandler_(belongingRtcpHandler) {
  (void)belongingRtcpHandler_;
}

int RtcpSenderReport::HandleSenderReport(const RtcpHeader& chead) {
  uint8_t blockCount = chead.getBlockCount();
  uint16_t srLen = chead.getRealLength();
  uint32_t ssrc = chead.getSSRC();
  uint64_t senderNtpTimestamp = chead.getNtpTimestamp();
  uint32_t senderRtpTimestamp = chead.getRtpTimestamp();
  uint32_t sendPkgs = chead.getPacketsSent();  // sender send packages sum
  uint32_t sendOcts = chead.getOctetsSent();   // sender send bytes sum

  tylog(
      "SR ssrc[%u] blockCount[%u] srLen[%u] ntpTimestamp=%s, rtpTimestamp[%u] "
      "sendPkgs[%u] sendOcts[%u]",
      ssrc, blockCount, srLen, NtpTime(senderNtpTimestamp).ToString().data(),
      senderRtpTimestamp, sendPkgs, sendOcts);

  // 存储sr用于计算rr反馈
  // to check map key number
  SrPkgInfo& info = this->belongingRtcpHandler_.belongingPeerConnection_
                        .rtpHandler_.ssrcInfoMap_.at(ssrc)
                        .srInfo_;
  info.recvMs = g_now_ms;
  info.SRCount++;
  info.SSRC = ssrc;
  info.blockCount = blockCount;
  info.srLen = srLen;
  info.NTPTimeStamps = senderNtpTimestamp;
  info.RTPTimeStamps = senderRtpTimestamp;
  info.sentPkgs = sendPkgs;
  info.sentOctets = sendOcts;

  return 0;
}

int RtcpSenderReport::CreateSenderReport(std::vector<char>* io_rtcpBin) {
  SrPkgInfo srPkgInfo{};

  // taylor mock
  srPkgInfo.SSRC = kDownlinkVideoSsrc;
  // for calculate RTT
  srPkgInfo.NTPTimeStamps = MsToNtp(g_now_ms).GetValue();
  srPkgInfo.RTPTimeStamps = 999888;
  srPkgInfo.sentOctets = 10000;
  srPkgInfo.sentPkgs = 20000;
  tylog("[srPkgInfo]SSRC:%u, NTP TimeStamps:%lu, RTP TimeStamps:%u",
        srPkgInfo.SSRC, srPkgInfo.NTPTimeStamps, srPkgInfo.RTPTimeStamps);

  RtcpHeader senderReport;
  senderReport.setPacketType(RtcpPacketType::kSenderReport);
  senderReport.setSSRC(srPkgInfo.SSRC);
  senderReport.setNtpTimestamp(srPkgInfo.NTPTimeStamps);
  senderReport.setRtpTimestamp(srPkgInfo.RTPTimeStamps);
  senderReport.setPacketsSent(srPkgInfo.sentPkgs);
  senderReport.setOctetsSent(srPkgInfo.sentOctets);
  senderReport.setLength(6);

  tylog("create sr=%s.", senderReport.ToString().data());

  char* buf = reinterpret_cast<char*>(&senderReport);
  io_rtcpBin->insert(io_rtcpBin->end(), buf,
                     buf + senderReport.getRealLength());

  return 0;
}

}  // namespace tywebrtc