// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtcp_receiver_report.h"

#include "src/global_tmp/global_tmp.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

RtcpReceiverReport::RtcpReceiverReport(RtcpHandler& belongingRtcpHandler)
    : belongingRtcpHandler_(belongingRtcpHandler) {}

// ref
// https://source.chromium.org/chromium/chromium/src/+/main:third_party/webrtc/modules/rtp_rtcp/source/rtcp_receiver.cc;l=576
int RtcpReceiverReport::HandleReceiverReport(const RtcpHeader& chead) {
  uint8_t block_count = chead.getBlockCount();
  tylog("recv RR block_count=%d.", block_count);
  uint16_t rr_length = (chead.getLength() + 1) * 4;
  uint32_t ssrc = chead.getSSRC();

  // RR report block is 24 B
  // https://datatracker.ietf.org/doc/html/rfc3550#section-6.4.2
  // |                 SSRC_1 (SSRC of first source)                 |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // | fraction lost |       cumulative number of packets lost       |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |           extended highest sequence number received           |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                      interarrival jitter                      |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                         last SR (LSR)                         |
  // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  // |                   delay since last SR (DLSR)                  |
  const int kRRReportBlockSize = sizeof(RtcpHeader::report_t::receiverReport_t);
  static_assert(24 == kRRReportBlockSize,
                "RRReportBlockSize size should be 24");

  const int kRTCPHeadSize = 8;
  if (kRTCPHeadSize + block_count * kRRReportBlockSize != rr_length) {
    tylog("[ReceiverReport] ssrc[%u] BlockCount[%u] rrLen[%u], invalid RR!",
          ssrc, block_count, rr_length);
    return -1;
  }

  int i;
  // hack: pDummyHead points to `last SR (LSR)` of last block,
  // cannot get RTCP head field.
  const RtcpHeader* pDummyHead;
  for (i = 0, pDummyHead = &chead; i < block_count; ++i,
      pDummyHead = reinterpret_cast<const RtcpHeader*>(
          reinterpret_cast<const char*>(pDummyHead) + kRRReportBlockSize)) {
    uint32_t sourceSsrc = pDummyHead->getSourceSSRC();
    // Fraction Lost
    uint8_t fractLost = pDummyHead->getFractionLost();
    // cumulative number of packets lost
    uint32_t lostPkgs = pDummyHead->getLostPackets();
    uint16_t seqnumCycles = pDummyHead->getSeqnumCycles();
    uint16_t highestSeq = pDummyHead->getHighestSeqnum();
    uint32_t jitter = pDummyHead->getJitter();
    const uint32_t kLSR = pDummyHead->getLastSr();
    const uint32_t kDLSR = pDummyHead->getDelaySinceLastSr();

    if (kLSR > 0 && kDLSR > 0) {
      NtpTime nowNtp = MsToNtp(g_now_ms);
      uint32_t rttNtp = CompactNtp(nowNtp) - kDLSR - kLSR;
      int rttMs = CompactNtpRttToMs(rttNtp);
      tylog("taylor in RR rtt ms=%d.", rttMs);
      // SaveDownLost(sourceSsrc, fractLost, lostPkgs, rtt);
      this->belongingRtcpHandler_.belongingPeerConnection_.signalHandler_
          .S2CReportRTT(rttMs);
    }

    tylog(
        "RR sourceSsrc[%u] fractLost[%u] lostPkgs[%u] seqnumCycles[%u] "
        "highestSeq[%u] jitter[%u] LSR[%u] DLSR[%u].",
        sourceSsrc, fractLost, lostPkgs, seqnumCycles, highestSeq, jitter, kLSR,
        kDLSR);

    RrPkgInfo& info = this->ssrcRRInfo[sourceSsrc];
    info.svrTimeMS = g_now_ms;
    info.RRCount++;
    info.fractionLost = fractLost;
  }

  return 0;
}

static int createRR(char* outBuf, int outBufLen, const RrPkgInfo& rrPkgInfo) {
  RtcpHeader receiverReport;
  receiverReport.setPacketType(RtcpPacketType::kReceiverReport);
  receiverReport.setSSRC(rrPkgInfo.sinkSSRC);
  receiverReport.setSourceSSRC(rrPkgInfo.sourceSSRC);
  receiverReport.setHighestSeqnum(rrPkgInfo.extendedSeq);
  receiverReport.setSeqnumCycles(rrPkgInfo.extendedSeq >> 16);
  receiverReport.setLostPackets(rrPkgInfo.lostPkgNum);
  receiverReport.setFractionLost(rrPkgInfo.fractionLost);
  receiverReport.setJitter(rrPkgInfo.jitter);
  receiverReport.setDelaySinceLastSr(rrPkgInfo.delaySinceLast);
  receiverReport.setLastSr(rrPkgInfo.lastSr);
  receiverReport.setLength(7);
  receiverReport.setBlockCount(1);

  char* buf = reinterpret_cast<char*>(&receiverReport);
  int len = (receiverReport.getLength() + 1) * 4;

  if (outBufLen < len) {
    return -1;
  }

  memcpy(outBuf, buf, len);

  return len;
}

int RtcpReceiverReport::CreateReceiverReport() {
  int ret = 0;

  // taylor check
  int remote_ssrc = this->belongingRtcpHandler_.belongingPeerConnection_
                        .rtpHandler_.upVideoSSRC;
  int uiLocSsrc = 23333;

  RrPkgInfo rrPkgInfo{};
  assert(rrPkgInfo.sinkSSRC == 0);
  assert(rrPkgInfo.lostPkgNum == 0);
  assert(rrPkgInfo.jitter == 0);
  assert(rrPkgInfo.lastSr == 0);
  rrPkgInfo.sinkSSRC = uiLocSsrc;
  rrPkgInfo.sourceSSRC = remote_ssrc;
  rrPkgInfo.extendedSeq = 0;   // taylor fix
  rrPkgInfo.fractionLost = 0;  // taylor fix
  rrPkgInfo.lostPkgNum = 0;    // taylor fix
  rrPkgInfo.jitter = 3;        // taylor fix
  rrPkgInfo.delaySinceLast = 0;
  rrPkgInfo.lastSr = 0;

  // taylor fixme
  NtpTime rcvNtp = MsToNtp(g_now_ms - 500);
  NtpTime nowNtp = MsToNtp(g_now_ms);
  rrPkgInfo.delaySinceLast = CompactNtp(nowNtp) - CompactNtp(rcvNtp);
  rrPkgInfo.lastSr = 23333;
  tylog("rr PkgInfo.delaySinceLast=%u, rr PkgInfo.lastSr=%u",
        rrPkgInfo.delaySinceLast, rrPkgInfo.lastSr);

  char outBuf[2048]{};
  int len = createRR(outBuf, 2048, rrPkgInfo);
  assert(len > 0);

  const uint64_t NTP = nowNtp.GetValue();
  uint64_t netNtp = (((uint64_t)ntohl(NTP)) << 32) + ntohl(NTP >> 32);

  // move to XR cpp?
  RtcpHeader rrtrReport;
  rrtrReport.setPacketType(RtcpPacketType::kExtendedReports);
  rrtrReport.setLength(4);
  rrtrReport.setSSRC(this->belongingRtcpHandler_.belongingPeerConnection_
                         .rtpHandler_.upVideoSSRC);

  rrtrReport.setBlockType(EnXRBlockType::kXRBlockRRTR);
  const int kRRTRBlockLen = 2;  // all block is (2+1)*4B
  rrtrReport.setBlockLen(kRRTRBlockLen);
  tylog(
      "NTPMS[%lu] NTP[%lu] "
      "middleNtp[%u],1 rr PkgInfo.fractionLost[%d]",
      nowNtp.ToMs(), NTP, CompactNtp(nowNtp),
      rrPkgInfo.fractionLost * 100 / 256);
  memcpy(outBuf + len, reinterpret_cast<char*>(&rrtrReport), 12);
  len += 12;
  // OPT: use setRrtrNtp
  memcpy(outBuf + len, reinterpret_cast<char*>(&netNtp),
         8);  // for calculate RTT
  len += 8;

  std::vector<char> rtcpBin(outBuf, outBuf + len);  // can use string view
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

  // *averageBitrate5Sec = (uint32_t)((9 * (*averageBitrate5Sec) +
  // outParaRFC3550A3.bit_rate) / 10);

  return 0;
}

}  // namespace tywebrtc