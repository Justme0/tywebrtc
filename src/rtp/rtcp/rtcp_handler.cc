// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// doc https://datatracker.ietf.org/doc/html/rfc3550

#include "src/rtp/rtcp/rtcp_handler.h"

#include <cassert>

#include "tylib/ip/ip.h"
#include "tylib/time/timer.h"

#include "src/log/log.h"
#include "src/pc/peer_connection.h"
#include "src/rtp/rtcp/rtcp_parser.h"

namespace tywebrtc {

extern int g_sock_fd;

RtcpHandler::RtcpHandler(PeerConnection &pc)
    : senderReport_(*this),
      receiverReport_(*this),
      nack_(*this),
      pli_(*this),
      extendedReport_(*this),
      belongingPeerConnection_(pc) {}

// 处理解密后的 RTCP 包
int RtcpHandler::HandleRtcpPacket(const std::vector<char> &vBufReceive) {
  // if peer doesn't exist, not handle it.
  // OPT: handle RRTR, RR, SR ...
  auto peerPC = belongingPeerConnection_.FindPeerPC();
  // if pull rtmp/srt/... stream, it's null;
  // if pull another webrtc, it's not null.

  int ret = 0;

  const char *movingBuf = vBufReceive.data();
  int rtcpLen = 0;
  size_t totalLen = 0;
  int index = 0;

  //可能是rtcp组合包
  while (true) {
    // OPT: refactor condition check
    if (totalLen >= vBufReceive.size()) {
      break;
    }

    movingBuf += rtcpLen;
    const RtcpHeader &chead = *reinterpret_cast<const RtcpHeader *>(movingBuf);
    rtcpLen = (chead.getLength() + 1) * 4;
    totalLen += rtcpLen;

    tylog(
        "recv number #%d rtcp=%s. rtcpLen=%u, totalLen=%zu. (input buf "
        "len=%zu)",
        index, chead.ToString().data(), rtcpLen, totalLen, vBufReceive.size());
    ++index;

    if (totalLen > vBufReceive.size()) {
      // should check why
      tylog("NOTE: totalLen=%zu > all input len=%zu, break", totalLen,
            vBufReceive.size());

      break;
    }

    // TODO: use virtual function? instead of switch
    switch (chead.getPacketType()) {
      case RtcpPacketType::kSenderReport: {
        ret = this->senderReport_.HandleSenderReport(chead);
        if (ret) {
          tylog("handle SR ret=%d.", ret);
          assert(!"shit");
          return ret;
        }
        break;
      }

      case RtcpPacketType::kReceiverReport: {
        ret = this->receiverReport_.HandleReceiverReport(chead);
        if (ret) {
          tylog("handle RR ret=%d.", ret);
          assert(!"shit");
          return ret;
        }
        break;
      }

      case RtcpPacketType::kSourceDescription: {
        tylog("recv SourceDescription block");
        break;
      }

      case RtcpPacketType::kBye: {
        tylog("recv bye block");
        break;
      }

      case RtcpPacketType::kApplicationDefined: {
        tylog("recv App block");
        break;
      }

      case RtcpPacketType::kGenericRtpFeedback: {
        switch (static_cast<RtcpGenericFeedbackFormat>(chead.getBlockCount())) {
          case RtcpGenericFeedbackFormat::kFeedbackNack: {
            ret = this->nack_.HandleNack(chead);
            if (ret) {
              tylog("handleNack ret=%d", ret);

              return ret;
            }

            // taylor OPT: should not return, should transfer other type of
            // packet
            return 0;

            break;
          }
          case RtcpGenericFeedbackFormat::kFeedbackTCC: {
            tylog("TCC feedback rtcp pkt.");

            break;
          }
          default:
            // should only log
            assert(!"unknown rtcp");
            break;
        }
        break;
      }

      case RtcpPacketType::kPayloadSpecificFeedback: {
        // TODO: move to ToString, use virtual function?
        tylog("specific feedback recv type [%s]",
              RtcpPayloadSpecificFormatToString(
                  static_cast<RtcpPayloadSpecificFormat>(chead.getBlockCount()))
                  .data());

        switch (static_cast<RtcpPayloadSpecificFormat>(chead.getBlockCount())) {
          case RtcpPayloadSpecificFormat::kRtcpPLI:
          case RtcpPayloadSpecificFormat::kRtcpSLI:
          case RtcpPayloadSpecificFormat::kRtcpFIR: {
            if (nullptr == peerPC) {
              tylog(
                  "another peerPC null(may only pull rtmp/srt/...), can not "
                  "req I frame.");

              break;
            }

            if (peerPC->rtpHandler_.upVideoSSRC == 0) {
              tylog(
                  "peerPC upVideo SSRC=0(maybe in begin time), so not transfer "
                  "video req to it. peerPC=%s.",
                  peerPC->ToString().data());

              // should continue handle other rtcp?
              return 0;
            }

            const_cast<RtcpHeader &>(chead).setSourceSSRC(
                peerPC->rtpHandler_.upVideoSSRC);

            // OPT: handle other type of RTCP source ssrc
            // if (rsphead->getSourceSSRC() == kDownlinkAudioSsrc) {
            //   assert(peerPC->rtpHandler_.upAudioSSRC != 0);
            //   rsphead->setSourceSSRC(peerPC->rtpHandler_.upAudioSSRC);
            // } else if (rsphead->getSourceSSRC() == kDownlinkVideoSsrc) {
            //   assert(peerPC->rtpHandler_.upVideoSSRC != 0);
            //   rsphead->setSourceSSRC(peerPC->rtpHandler_.upVideoSSRC);
            // }

            break;
          }

          case RtcpPayloadSpecificFormat::kRtcpRPSI:
            break;

          case RtcpPayloadSpecificFormat::kRtcpREMB:
            break;

          default:
            // should only log
            assert(!"unknown rtcp");
            break;
        }
        break;
      }

      case RtcpPacketType::kExtendedReports: {
        ret = this->extendedReport_.HandleExtendedReports(chead);
        if (ret) {
          tylog("handle SR ret=%d.", ret);
          assert(!"shit");
          return ret;
        }
        break;
      }

      default:
        // should only log
        assert(!"unknown rtcp payload type");
        break;
    }
  }

  if (nullptr == peerPC) {
    tylog("another peerPC null, not transparent transfer.");

    return 0;
  }

  DumpSendPacket(vBufReceive);
  ret = peerPC->srtpHandler_.ProtectRtcp(
      const_cast<std::vector<char> *>(&vBufReceive));
  if (ret) {
    tylog("downlink protect rtcp ret=%d", ret);
    return ret;
  }

  // unvarnished transmission to peer
  ret = peerPC->SendToClient(vBufReceive);
  if (ret) {
    tylog("send ret=%d", ret);
    return ret;
  }

  return 0;
}

}  // namespace tywebrtc