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
    : belongingRtcpHandler_(belongingRtcpHandler) {}

int RtcpSenderReport::HandleSenderReport(const RtcpHeader& chead) {
  uint8_t blockCount = chead.getBlockCount();
  uint16_t srLen = (chead.getLength() + 1) * 4;
  uint32_t ssrc = chead.getSSRC();
  uint64_t ntpTimestamp = chead.getNtpTimestamp();
  uint32_t rtpTimestamp = chead.getRtpTimestamp();
  uint32_t sendPkgs = chead.getPacketsSent();  // sender send packages sum
  uint32_t sendOcts = chead.getOctetsSent();   // sender send bytes sum

  tylog(
      "sender report ssrc[%u] blockCount[%u] srLen[%u] ntpTimestamp[%lu]"
      "rtpTimestamp[%u] sendPkgs[%u] sendOcts[%u] ",
      ssrc, blockCount, srLen, ntpTimestamp, rtpTimestamp, sendPkgs, sendOcts);

  // 存储sr用于计算rr反馈
  // to check map key number
  SrPkgInfo& info = this->ssrcSRInfo[ssrc];
  info.svrTimeMS = g_now_ms;
  info.SRCount++;
  info.SSRC = ssrc;
  info.blockCount = blockCount;
  info.srLen = srLen;
  info.NTPTimeStamps = NtpTime(ntpTimestamp).ToMs();
  info.RTPTimeStamps = rtpTimestamp;
  info.sentPkgs = sendPkgs;
  info.sentOctets = sendOcts;

  return 0;
}

static int createSR(char* outBuf, int outBufLen, SrPkgInfo srPkgInfo) {
  RtcpHeader senderReport;
  senderReport.setPacketType(RtcpPacketType::kSenderReport);
  senderReport.setSSRC(srPkgInfo.SSRC);
  senderReport.setNtpTimestamp(srPkgInfo.NTPTimeStamps);
  senderReport.setRtpTimestamp(srPkgInfo.RTPTimeStamps);
  senderReport.setPacketsSent(srPkgInfo.sentPkgs);
  senderReport.setOctetsSent(srPkgInfo.sentOctets);
  senderReport.setLength(6);

  char* buf = reinterpret_cast<char*>(&senderReport);
  int len = (senderReport.getLength() + 1) * 4;

  if (outBufLen < len) {
    return -1;
  }

  memcpy(outBuf, buf, len);

  return len;
}

static int createDLRR(char* outBuf, int outBufLen, uint32_t ssrc,
                      uint32_t rrSsrc, uint32_t lastRrNtp, uint32_t dlrrNtp) {
  RtcpHeader dlrr;
  dlrr.setPacketType(RtcpPacketType::kExtendedReports);
  dlrr.setSSRC(ssrc);
  dlrr.setLength(5);
  dlrr.setBlockCount(1);

  dlrr.setBlockType(EnXRBlockType::kXRBlockDLRR);
  dlrr.setBlockLen(3);
  dlrr.setRrSsrc(rrSsrc);
  dlrr.setLastRr(lastRrNtp);
  dlrr.setDLRR(dlrrNtp);

  char* buf = reinterpret_cast<char*>(&dlrr);
  int len = (dlrr.getLength() + 1) * 4;

  if (outBufLen < len) {
    return -1;
  }

  memcpy(outBuf, buf, len);

  return len;
}

int RtcpSenderReport::CreateSenderReport() {
  int ret = 0;

  SrPkgInfo videoSrPkgInfo;

  // taylor mock
  NtpTime nowNtp = MsToNtp(g_now_ms);
  videoSrPkgInfo.SSRC = kDownlinkVideoSsrc;
  videoSrPkgInfo.NTPTimeStamps = nowNtp.GetValue();  // for calculate RTT
  videoSrPkgInfo.RTPTimeStamps = 999888;
  videoSrPkgInfo.sentOctets = 10000;
  videoSrPkgInfo.sentPkgs = 20000;

  tylog("[videoSrPkgInfo]SSRC:%u, NTP TimeStamps:%lu, RTP TimeStamps:%u",
        videoSrPkgInfo.SSRC, videoSrPkgInfo.NTPTimeStamps,
        videoSrPkgInfo.RTPTimeStamps);

  char outBuf[2048]{};
  int len = createSR(outBuf, 2048, videoSrPkgInfo);
  if (len <= 0) {
    tylog("video create SR Failed!");
    assert(!"buffer too small, to use vector");

    return -1;
  }

  // taylor FIXME
  uint64_t relayNtp =
      MsToNtp(g_now_ms).GetValue() - MsToNtp(g_now_ms - 500).GetValue();
  uint32_t dlrrNtp = CompactNtp(NtpTime(relayNtp));

  const int rrtr_ssrc = 1;   // ?
  uint32_t lastRrtrNtp = 3;  // test
  const int dlrrLen = createDLRR(outBuf + len, 2048 - len, kDownlinkVideoSsrc,
                                 rrtr_ssrc, lastRrtrNtp, dlrrNtp);
  tylog("send dlrr, ssrc:%u, rrtrSsrc:%u, lastRrtrNtp:%u, dlrr Ntp:%u",
        kDownlinkVideoSsrc, rrtr_ssrc, lastRrtrNtp, dlrrNtp);

  if (dlrrLen > 0) {
    len += dlrrLen;
  } else {
    tylog("send dlrr,video create DLRR Failed!");
    assert(!"should not use assert :)");

    return -2;
  }

  // OPT: use string view
  std::vector<char> rtcpBin(outBuf, outBuf + len);
  DumpSendPacket(rtcpBin);
  ret = this->belongingRtcpHandler_.belongingPeerConnection_.srtpHandler_
            .ProtectRtcp(const_cast<std::vector<char>*>(&rtcpBin));
  if (ret) {
    tylog("send to client, protect rtcp ret=%d", ret);

    return ret;
  }
  ret = this->belongingRtcpHandler_.belongingPeerConnection_.SendToClient(
      rtcpBin);
  if (ret) {
    tylog("send to client ret=%d", ret);

    return ret;
  }

  return 0;
}

}  // namespace tywebrtc