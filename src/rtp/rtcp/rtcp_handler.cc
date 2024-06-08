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
      psfb_(*this),
      rtpfb_(*this),
      extendedReport_(*this),
      belongingPC_(pc) {}

// 处理解密后的 RTCP 包
int RtcpHandler::HandleRtcpPacket(const std::vector<char> &vBufReceive) {
  // if peer doesn't exist, not handle it.
  // OPT: handle RRTR, RR, SR ...
  // auto peerPC = belongingPC_.FindPeerPC();
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
    rtcpLen = chead.getRealLength();
    totalLen += rtcpLen;

    tylog("recv number #%d rtcp=%s, totalLen=%zu (input buf len=%zu)", index,
          chead.ToString().data(), totalLen, vBufReceive.size());
    ++index;

    if (totalLen > vBufReceive.size()) {
      // should check why
      tylog("NOTE: totalLen=%zu > all input len=%zu, break", totalLen,
            vBufReceive.size());

      assert(totalLen - vBufReceive.size() <= 10);

      break;
    }

    // TODO: use virtual function? instead of switch
    switch (chead.getPacketType()) {
      case RtcpPacketType::kSenderReport: {
        ret = this->senderReport_.HandleSenderReport(chead);
        if (ret) {
          tylog("handle SR ret=%d.", ret);
          assert(!"should not user assert :)");
          return ret;
        }
        break;
      }

      case RtcpPacketType::kReceiverReport: {
        ret = this->receiverReport_.HandleReceiverReport(chead);
        if (ret) {
          tylog("handle RR ret=%d.", ret);
          assert(!"should not user assert :)");
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

      case RtcpPacketType::kRtpFeedback: {
        ret = this->rtpfb_.HandleRtpFeedback(chead);
        if (ret) {
          tylog("handle rtp fb ret=%d.", ret);
          assert(!"should not user assert :)");
          return ret;
        }
        break;
      }

      case RtcpPacketType::kPayloadSpecificFeedback: {
        ret = this->psfb_.HandlePayloadSpecificFeedback(chead);
        if (ret) {
          tylog("handle ps fb ret=%d.", ret);
          assert(!"should not user assert :)");
          return ret;
        }
        break;
      }

      case RtcpPacketType::kExtendedReports: {
        ret = this->extendedReport_.HandleExtendedReports(chead);
        if (ret) {
          tylog("handle SR ret=%d.", ret);
          assert(!"should not user assert :)");
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

  /*
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
    */

  return 0;
}

}  // namespace tywebrtc