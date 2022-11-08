#include "rtp/rtp_handler.h"

#include <arpa/inet.h>

#include <cassert>
#include <cstring>
#include <string>

#include "tylib/ip/ip.h"
#include "tylib/string/any_to_string.h"

#include "global_tmp/global_tmp.h"
#include "log/log.h"
#include "pc/peer_connection.h"
#include "rtp/pack_unpack/rtp_to_h264.h"
#include "rtp/rtcp_parser.h"
#include "rtp/rtp_parser.h"

// string enum, for print convenience
const std::string kMediaTypeRtcp = "rtcp";
const std::string kMediaTypeVideo = "video";
const std::string kMediaTypeAudio = "audio";

RtpHandler::RtpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

extern int g_sock_fd;

// move to tylib
// static void printAscii(const std::string &s) {
//   char arr[9000]{};
//   char *p = arr;
//
//   for (char c : s) {
//     p += sprintf(p, "%X ", c);
//   }
//   tylog("%s", arr);
// }

int DumpPacketH264(const std::vector<char> &packet,
                   H264Unpacketizer &unpacker) {
  int ret = 0;
  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(packet.data());

  std::string mediaType;
  ret = RtpRtcpStrategy::GetMediaType(packet, &mediaType);
  if (ret) {
    tylog("get media type fail, ret=%d", ret);
    return ret;
  }

  if (mediaType == kMediaTypeVideo) {
    std::vector<MediaData> media;
    ret = unpacker.Unpacketize(packet, &media);
    if (ret) {
      tylog("unpacketize rtp ret=%d", ret);
      return ret;
    }

    tylog("unpack media.size=%zu", media.size());
    for (const MediaData &m : media) {
      ret = unpacker.DumpRawStream(m.data_, rtpHeader.getSSRC());
      if (ret) {
        tylog("dump ret=%d", ret);
        return ret;
      }
    }
  }

  return 0;
}

int RtpHandler::SendToPeer_(std::vector<char> &packet) {
  int ret = 0;
  RtpHeader &downlinkRtpHeader = *reinterpret_cast<RtpHeader *>(packet.data());

  std::string mediaType;
  ret = RtpRtcpStrategy::GetMediaType(packet, &mediaType);
  if (ret) {
    tylog("get media type fail, ret=%d", ret);
    return ret;
  }

  tylog("downlink send media type=%s.", mediaType.data());

  if (mediaType == kMediaTypeAudio) {
    // taylor audio not use pacing
    const int kDownlinkAudioSsrc = 16854838;  // taylor to make dynamic
    downlinkRtpHeader.setSSRC(kDownlinkAudioSsrc);

    const int kDownlinkAudioPayloadType = 111;
    downlinkRtpHeader.setPayloadType(kDownlinkAudioPayloadType);
  } else if (mediaType == kMediaTypeVideo) {
    const int kDownlinkVideoSsrc = 33697348;  // taylor to make dynamic
    downlinkRtpHeader.setSSRC(kDownlinkVideoSsrc);

    const int kDownlinkVideoPayloadType = 125;  // H.264
    downlinkRtpHeader.setPayloadType(kDownlinkVideoPayloadType);
  } else {
    tylog("invalid downlink send media type=%s.", mediaType.data());
    assert(!"invalid media type");
  }

  ret = this->belongingPeerConnection_.srtpHandler_.ProtectRtp(
      const_cast<std::vector<char> *>(&packet));
  if (ret) {
    tylog("downlink protect rtp ret=%d", ret);
    return ret;
  }

  sockaddr_in addr =
      tylib::ConstructSockAddr(this->belongingPeerConnection_.clientIP_,
                               this->belongingPeerConnection_.clientPort_);
  ssize_t sendtoLen =
      sendto(g_sock_fd, packet.data(), packet.size(), 0,
             reinterpret_cast<sockaddr *>(&addr), sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    return -1;
  }
  tylog("sendto reply succ buf size=%ld, ip=%s, port=%d.", sendtoLen,
        belongingPeerConnection_.clientIP_.data(),
        belongingPeerConnection_.clientPort_);

  return 0;
}

// use OO?
int RtpHandler::HandleRtcpPacket_(const std::vector<char> &vBufReceive) {
  int ret = 0;

  const RtcpHeader &rtcpHeader =
      *reinterpret_cast<const RtcpHeader *>(vBufReceive.data());
  tylog("recv rtcp=%s", rtcpHeader.ToString().data());

  /*
    RtcpHeader *chead = reinterpret_cast<RtcpHeader *>(pPackage);
    if (!chead->isRtcp()) {
      tylog("pkg is not rtcp, pt=%d", chead->packettype);
      return -1;
    }

    unsigned int subType = 0;
    STVideoUserInfo *pWatcherUser = nullptr;

    // 行使用远端ssrc判断
    if (m_pTWebRTCUserInfo->ullTinyId == m_pc->ullTinyId) {
      if (chead->getSSRC() == m_pc->tVideoUserInfo[0].uiRemoteSsrc)  //视频SR包
      {
        pWatcherUser = &m_pc->tVideoUserInfo[0];
        subType = DT_BIGVIDIO;
      } else if (chead->getSSRC() == m_pc->tVideoUserInfo[1].uiRemoteSsrc) {
        pWatcherUser = &m_pc->tVideoUserInfo[1];
      } else if (chead->getSSRC() == m_pc->tAudioInfo.uiRemoteSsrc)  //音频SR包
      {
        subType = DT_AUDIO;
      }
    } else {
      if (chead->getSourceSSRC() == m_pc->tVideoUserInfo[0].uiLocalSsrc) {
        subType = DT_BIGVIDIO;
        pWatcherUser = &m_pc->tVideoUserInfo[0];
      } else if (chead->getSourceSSRC() == m_pc->tVideoUserInfo[1].uiLocalSsrc)
    { pWatcherUser = &m_pc->tVideoUserInfo[1]; } else if (chead->getSourceSSRC()
    == m_pc->tAudioInfo.uiLocalSsrc) { subType = DT_AUDIO;
      }
    }

    if (0 == subType) {
      tylor(
          "Rtcp packettype:%u BlockCount:%u subType:%u\r\n Rtcp[SSrc:%u "
          "SrcSSRC:%u] \r\nVid[RemSsrc:%u LocSsrc:%u] \r\nAud[RemSsrc:%u "
          "LocSsrc:%u]",
          chead->packettype, chead->getBlockCount(), subType, chead->getSSRC(),
          chead->getSourceSSRC(), m_pc->tVideoUserInfo[0].uiRemoteSsrc,
          m_pc->tVideoUserInfo[0].uiLocalSsrc, m_pc->tAudioInfo.uiRemoteSsrc,
          m_pc->tAudioInfo.uiLocalSsrc);
    }

    char *movingBuf = pPackage;
    int rtcpLen = 0;
    int totalLen = 0;
    while (true)  //可能是rtcp组合包
    {
      if (totalLen >= iLen) break;

      movingBuf += rtcpLen;
      chead = reinterpret_cast<RtcpHeader *>(movingBuf);
      rtcpLen = (chead->getLength() + 1) * 4;
      totalLen += rtcpLen;

      tylor("rtcpLen[%u], totalLen[%u], iLen[%u]", rtcpLen, totalLen,
                         iLen);

      if (totalLen > iLen) break;

      if (chead->packettype == RTCP_RTP_Feedback_PT)  // NACK  RFC3550
      {
        if (chead->getBlockCount() == 1) {
          HandleNack(chead);
        } else if (chead->getBlockCount() == RTCP_AFB) {
          // REMB只会有在房间只有两个web的时候透传
          if (IsByPassMod()) {
            RelayRtcpReq(movingBuf, rtcpLen, subType);
          }

          tylor(
              "REMB[Recv]:  BitRate[%lu] packettype:%u BlockCount:%u "
              "subType:%u\r\n Rtcp[SSrc:%u SrcSSRC:%u] \r\nVid[RemSsrc:%u "
              "LocSsrc:%u] \r\nAud[RemSsrc:%u LocSsrc:%u]",
              chead->getREMBBitRate(), chead->packettype,
    chead->getBlockCount(), subType, chead->getSSRC(), chead->getSourceSSRC(),
              m_pc->tVideoUserInfo[0].uiRemoteSsrc,
              m_pc->tVideoUserInfo[0].uiLocalSsrc,
    m_pc->tAudioInfo.uiRemoteSsrc, m_pc->tAudioInfo.uiLocalSsrc);
        }

      } else if (chead->packettype == RTCP_PS_Feedback_PT) {
        // I帧申请透传
        if ((RTCP_PLI_FMT == chead->getBlockCount()) ||
            (RTCP_SLI_FMT == chead->getBlockCount()) ||
            (RTCP_FIR_FMT == chead->getBlockCount())) {
          if (m_pTWebRTCUserInfo->ullTinyId == m_pc->ullTinyId) {
            continue;
          }

          tylor(
              "PLI: MediaMode:%u ClientType:%u Rtcp packettype:%u BlockCount:%u
    " "subType:%u\r\n Rtcp[SSrc:%u SrcSSRC:%u] \r\nVid[RemSsrc:%u " "LocSsrc:%u]
    \r\nAud[RemSsrc:%u LocSsrc:%u]", m_pTWebRTCUserInfo->MediaMode,
    m_pc->ClientType, chead->packettype, chead->getBlockCount(), subType,
    chead->getSSRC(), chead->getSourceSSRC(),
    m_pc->tVideoUserInfo[0].uiRemoteSsrc, m_pc->tVideoUserInfo[0].uiLocalSsrc,
    m_pc->tAudioInfo.uiRemoteSsrc, m_pc->tAudioInfo.uiLocalSsrc);

          if (!pWatcherUser) {
            continue;
          }

          // TODO 位置修改
          pWatcherUser->PLICnt++;
          if (m_pc->ClientType != USER_TYPE_WEBRTC) {
            // TODO 位置修改 急速模式下sdk
            tylor("rtcp from web call RequestPeerIDRFrame");
            pWatcherUser->ReqSdkIFrFlag++;
            if (pWatcherUser->ReqSdkIFrFlag == 0) {
              pWatcherUser->ReqSdkIFrFlag = 1;
            }
            m_pTWebRTCUserInfo->RequestPeerIDRFrame(m_pc, pWatcherUser);
          } else if (m_pc->ClientType == USER_TYPE_WEBRTC) {
            // TODO 位置修改 急速模式下的web，非急速模式下的web
            pWatcherUser->ReqICnt++;
            // RelayRtcpReq(movingBuf,rtcpLen, subType);
            tylor("rtcp from web call RequestPeerIDRFrame");
            m_pTWebRTCUserInfo->RequestPeerIDRFrame(m_pc, pWatcherUser, true);
          }
        }
      } else if (chead->packettype == RTCP_Sender_PT)  // SR
      {
        //广播发送报告广播
        if (IsByPassMod()) {
          BroadcastRtcpReq(movingBuf, rtcpLen, subType);
        }

        tylor("[RTCP_Sender_PT]");
        HandleSR(chead);
      } else if (chead->packettype == RTCP_Receiver_PT)  // RR
      {
        //接收报告只会有在房间只有两个web的时候透传
        if (IsByPassMod()) {
          RelayRtcpReq(movingBuf, rtcpLen, subType);
        }

        tylor("[RTCP_Receiver_PT]");
        HandleRR(chead);
      } else if (chead->packettype == RTCP_XR_PT)  // XR  rfc3611
      {
        tylor("[ExtendedReport]");
        //广播发送报告广播
        if (IsByPassMod()) {
          BroadcastRtcpReq(movingBuf, rtcpLen, subType);
        }
        HandleXR(chead);
      }
    }
    */

  // taylor origin:

  // downlink

  // RtcpHeader &downlinkRtcpHeader = const_cast<RtcpHeader &>(rtcpHeader);

  /*
    if (mediaType == kMediaTypeAudio) {
      const int kDownlinkAudioSsrc = 16854838;  // taylor to make dynamic
      downlinkRtpHeader.setSSRC(kDownlinkAudioSsrc);

      const int kDownlinkAudioPayloadType = 111;
      downlinkRtpHeader.setPayloadType(kDownlinkAudioPayloadType);
    } else {
      const int kDownlinkVideoSsrc = 33697348;  // taylor to make dynamic
      downlinkRtpHeader.setSSRC(kDownlinkVideoSsrc);

      const int kDownlinkVideoPayloadType = 125;  // H.264
      downlinkRtpHeader.setPayloadType(kDownlinkVideoPayloadType);
    }
    */

  ret = this->belongingPeerConnection_.srtpHandler_.ProtectRtcp(
      const_cast<std::vector<char> *>(&vBufReceive));
  if (ret) {
    tylog("downlink protect rtcp ret=%d", ret);
    return ret;
  }

  sockaddr_in addr =
      tylib::ConstructSockAddr(this->belongingPeerConnection_.clientIP_,
                               this->belongingPeerConnection_.clientPort_);
  ssize_t sendtoLen = sendto(g_sock_fd, vBufReceive.data(), vBufReceive.size(),
                             0, reinterpret_cast<struct sockaddr *>(&addr),
                             sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    return -1;
  }
  tylog("sendto reply succ buf size=%ld, ip=%s, port=%d.", sendtoLen,
        belongingPeerConnection_.clientIP_.data(),
        belongingPeerConnection_.clientPort_);

  return 0;
}

int RtpHandler::HandleRtpPacket(const std::vector<char> &vBufReceive) {
  int ret = 0;

  // if we recv web's data, dtls should complete in Chrome
  // OPT: no need call hand shake complete function each time recv rtp
  const bool kSessionCompleted = true;
  ret = belongingPeerConnection_.dtlsHandler_.HandshakeCompleted(
      kSessionCompleted);
  if (ret) {
    tylog(
        "already recv rtp, we can handshakeCompleted safely, but ret=%d, but "
        "not return error",
        ret);
  }

  belongingPeerConnection_.stateMachine_ = EnumStateMachine::GOT_RTP;
  std::string mediaType;
  ret = RtpRtcpStrategy::GetMediaType(vBufReceive, &mediaType);
  if (ret) {
    tylog("get media type fail, ret=%d", ret);
    return ret;
  }

  tylog("receive %s", mediaType.data());

  // should refactor if else for media type

  if (mediaType == kMediaTypeRtcp) {
    ret = belongingPeerConnection_.srtpHandler_.UnprotectRtcp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("unprotect RTCP fail, ret=%d", ret);
      return ret;
    }

    ret = HandleRtcpPacket_(vBufReceive);
    if (ret) {
      tylog("HandleRtcpPacket fail, ret=%d", ret);
      return ret;
    }
  } else if (mediaType == kMediaTypeAudio || mediaType == kMediaTypeVideo) {
    tylog("before unprotect, paddinglen=%d", getRtpPaddingLength(vBufReceive));
    // reuse original buffer
    // taylor consider restart svr
    ret = belongingPeerConnection_.srtpHandler_.UnprotectRtp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("unprotect RTP (not RTCP) fail ret=%d", ret);
      return ret;
    }
    tylog("after unprotect, paddinglen=%d", getRtpPaddingLength(vBufReceive));

    const RtpHeader &rtpHeader =
        *reinterpret_cast<const RtpHeader *>(vBufReceive.data());
    tylog("recv rtp=%s (maybe out-of-order)", rtpHeader.ToString().data());

    SSRCInfo &ssrcInfo = this->ssrcInfoMap_[rtpHeader.getSSRC()];

    const uint16_t itemSeq = rtpHeader.getSeqNumber();
    int64_t itemCycle = 0;

    // monitor corner cass 12 packet
    if (itemSeq >= 65530 || itemSeq <= 5) {
      tylog("monitor corner case, packet=%s.", rtpHeader.ToString().data());
    }

    // update 3 var: ssrcInfo.biggestSeq, ssrcInfo.biggestCycle, itemCycle
    if (itemSeq == ssrcInfo.biggestSeq) {
      // todo more logic
      tylog("recv repeated rtp packet, ignoer it=%s.",
            rtpHeader.ToString().data());
      itemCycle = ssrcInfo.biggestCycle;
    } else if (itemSeq > ssrcInfo.biggestSeq) {
      if (AheadOf(itemSeq, ssrcInfo.biggestSeq)) {
        // case 1 recv newer, not rollback, most common case
        // ssrcInfo.biggestCycle not change
        ssrcInfo.biggestSeq = itemSeq;
        itemCycle = ssrcInfo.biggestCycle;
      } else {
        // case 2 recv old, but is last cycle
        if (ssrcInfo.biggestCycle <= 0) {
          // todo more logic, should return?
          tylog("recv shit packet, ignore it=%s.", rtpHeader.ToString().data());
          assert(!"should not reach here unless hacker attacks us, now we assert it");
        } else {
          itemCycle = ssrcInfo.biggestCycle - 1;  // notice
          tylog("recv old packet=%s. also old cycle",
                rtpHeader.ToString().data());
        }
      }
    } else {
      // itemSeq < ssrcInfo.biggestSeq
      // e.g. ssrcInfo.biggestSeq is 65535, itemSeq is 0
      if (AheadOf(itemSeq, ssrcInfo.biggestSeq)) {
        // case 3, recv newer, but rollback
        ssrcInfo.biggestSeq = itemSeq;
        ++ssrcInfo.biggestCycle;  // key
        itemCycle = ssrcInfo.biggestCycle;
        tylog("recv newer packet=%s. seq rollback",
              rtpHeader.ToString().data());
      } else {
        // case 4 recv old, same cycle
        tylog("recv old packet=%s.", rtpHeader.ToString().data());
        // ssrcInfo.biggestSeq, ssrcInfo.biggestCycle not change
        itemCycle = ssrcInfo.biggestCycle;
      }
    }

    // must save cycle for each packet, because may be different
    RtpBizPacket rtpBizPacket(
        std::move(const_cast<std::vector<char> &>(vBufReceive)), itemCycle);
    assert(vBufReceive.empty());

    // hack
    ssrcInfo.rtpReceiver.PushToJitter(std::move(rtpBizPacket));

    assert(vBufReceive.empty());

    std::vector<RtpBizPacket> orderedPackets =
        ssrcInfo.rtpReceiver.PopOrderedPackets();

    tylog("pop jitter's OrderedPackets size=%zu", orderedPackets.size());

    for (RtpBizPacket &packet : orderedPackets) {
      ret = DumpPacketH264(packet.rtpRawPacket, ssrcInfo.h264Unpacketizer);
      if (ret) {
        tylog("dump uplink h264 ret=%d", ret);
        return ret;
      }

      ssrcInfo.rtpSender.Enqueue(std::move(packet));
      assert(packet.rtpRawPacket.empty());
    }

    // downlink
    std::vector<RtpBizPacket> sendPackets = ssrcInfo.rtpSender.Dequeue();

    tylog("send to peer packets size=%zu", sendPackets.size());
    for (RtpBizPacket &packet : sendPackets) {
      ret = SendToPeer_(packet.rtpRawPacket);
      if (ret) {
        tylog("send to peer ret=%d", ret);
        return ret;
      }
    }
  } else {
    tylog("receive unknown type of data=%s, return", mediaType.data());

    // should not use assert
    assert(!"receive unknown media type, should already filter and return");

    return -2;
  }

  return 0;
}

std::string RtpHandler::ToString() const {
  return "rtpDummyData";
  // return tylib::format_string( "{ssrcMap=%s}",
  // tylib::AnyToString(ssrc2unpacker_).data());
}
