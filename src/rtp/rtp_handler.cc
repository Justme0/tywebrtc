#include "rtp/rtp_handler.h"

#include <arpa/inet.h>

#include <cassert>
#include <cstring>
#include <string>

#include "log/log.h"
#include "pc/peer_connection.h"
#include "rtp/codec_parser/rtp2h264.h"
#include "rtp/rtcp_parser.h"
#include "rtp/rtp_parser.h"

// string enum, for print convenience
const std::string kMediaTypeRtcp = "rtcp";
const std::string kMediaTypeVideo = "video";
const std::string kMediaTypeAudio = "audio";

RtpHandler::RtpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

extern int g_sock_fd;
extern struct sockaddr_in g_stConnAddr;  // to use data member

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
    STVideoUserInfo *pWatcherUser = NULL;

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

  ssize_t sendtoLen =
      sendto(g_sock_fd, vBufReceive.data(), vBufReceive.size(), 0,
             reinterpret_cast<struct sockaddr *>(&g_stConnAddr),
             sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    return -1;
  }
  tylog("sendto reply buf size=%ld", sendtoLen);

  return 0;
}

FILE *rtp_2_h264_file_;  // to move to 264 unpack class

int RtpHandler::HandleRtpPacket(const std::vector<char> &vBufReceive) {
  int ret = 0;

  // if we recv web's data, dtls should complete in Chrome
  // OPT: no need call hand shake complete function each time recv rtp
  const bool kSessionCompleted = true;
  // taylor no need call every time?
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
    tylog("recv rtp=%s", rtpHeader.ToString().data());

    if (mediaType == kMediaTypeVideo) {
      auto media =
          this->ssrc2unpacker[rtpHeader.getSSRC()].Unpacketize(vBufReceive);
      tylog("unpack media.size=%zu", media.size());
      for (auto &it : media) {
        // dump H.264
        if (rtp_2_h264_file_ == nullptr) {
          std::string f("./test_");
          f += "uplink";
          f += ".h264";
          rtp_2_h264_file_ = fopen(f.c_str(), "wb+");
        }

        fwrite(it->data_, it->len_, 1, rtp_2_h264_file_);
      }
    }

    // downlink

    RtpHeader &downlinkRtpHeader = const_cast<RtpHeader &>(rtpHeader);

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

    ret = this->belongingPeerConnection_.srtpHandler_.ProtectRtp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("downlink protect rtp ret=%d", ret);
      return ret;
    }

    ssize_t sendtoLen =
        sendto(g_sock_fd, vBufReceive.data(), vBufReceive.size(), 0,
               reinterpret_cast<struct sockaddr *>(&g_stConnAddr),
               sizeof(struct sockaddr_in));
    if (-1 == sendtoLen) {
      tylog("sendto errorno=%d[%s]", errno, strerror(errno));
      return -1;
    }
    tylog("sendto reply buf size=%ld", sendtoLen);
  } else {
    tylog("receive unknown type of data=%s, return", mediaType.data());

    // should not use assert
    assert(!"receive unknown media type, should already filter and return");

    return -2;
  }

  return 0;
}
