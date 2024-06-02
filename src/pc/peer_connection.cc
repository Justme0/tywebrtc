// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/pc/peer_connection.h"

#include "tylib/ip/ip.h"
#include "tylib/time/time_util.h"
#include "tylib/time/timer.h"

#include "src/log/log.h"

namespace tywebrtc {

PeerConnection::PeerConnection(const std::string &ip, int port)
    : stateMachine_(EnumStateMachine::GOT_CANDIDATE),  // sdp has candiate
      clientIP_(ip),
      clientPort_(port),
      sdpHandler_(*this),
      iceHandler_(*this),
      dtlsHandler_(*this, false),  // taylor 写死 dtls client
      rtpHandler_(*this),
      rtcpHandler_(*this),
      srtpHandler_(*this),
      dataChannelHandler_(*this),
      signalHandler_(*this),
      pullHandler_(*this),
      initTimeMs_(g_now_ms) {}

// vBufSend is encrypted data if RTP
int PeerConnection::SendToClient(const std::vector<char> &vBufSend) const {
  int r = rand() % 100;
  if (r < kDownlossRateMul100) {
    tylog("down rand=%d lostrate=%d%%, drop!", r, kDownlossRateMul100);

    return 0;
  }

  sockaddr_in addr = tylib::ConstructSockAddr(clientIP_, clientPort_);
  ssize_t sendtoLen =
      sendto(g_sock_fd, vBufSend.data(), vBufSend.size(), 0,
             reinterpret_cast<sockaddr *>(&addr), sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto ret=-1 errorno=%d[%s]", errno, strerror(errno));
    return -1;
  }
  tylog("sendto ret=%ld, bizDataLen=%zu, ip=%s, port=%d.", sendtoLen,
        vBufSend.size(), clientIP_.data(), clientPort_);

  assert(sendtoLen == static_cast<int>(vBufSend.size()));

  return 0;
}

int PeerConnection::HandlePacket(const std::vector<char> &vBufReceive) {
  this->lastActiveTimeMs_ = g_now_ms;

  int ret = 0;

  uint8_t cSubCmd = vBufReceive.front();
  PacketType packType = getPacketType(cSubCmd);
  tylog("subcmd=%hhu, packType=%s (if RTP, maybe RTCP)", cSubCmd,
        PacketTypeToString(packType).data());
  tylog("stateMachine=%s", StateMachineToString(stateMachine_).data());

  // packType number is little, so we don't use map-callback style, just
  // switch-case
  switch (packType) {
    case PacketType::STUN: {
      DumpRecvPacket(vBufReceive);
      ret = iceHandler_.HandleIcePacket(vBufReceive);
      if (ret) {
        tylog("handle ice packet fail, ret=%d", ret);
        return ret;
      }
      break;
    }

    case PacketType::DTLS: {
      DumpRecvPacket(vBufReceive);
      ret = dtlsHandler_.HandleDtlsPacket(vBufReceive);
      if (ret) {
        tylog("handle dtls packet fail, ret=%d", ret);
        return ret;
      }
      break;
    }

    case PacketType::RTP: {
      if (this->sdpHandler_.bNotUseSrtp) {
        if (this->stateMachine_ < EnumStateMachine::DTLS_DONE) {
          tylog(
              "first recv RTP, but not launch DTLS, maybe use "
              "--disable-webrtc-encryption, the check is not very strict :) "
              "should use signal.");

          // set to DTLS_DONE for rtp handler launch timer and others,
          // OPT
          this->stateMachine_ = EnumStateMachine::DTLS_DONE;
          tylog("set stateMachine to %s.",
                StateMachineToString(stateMachine_).data());
        }
      } else {
        // if we recv web's data, dtls should complete in Chrome
        // OPT: no need call hand shake complete function each time recv rtp
        const bool kSessionCompleted = true;
        ret = dtlsHandler_.HandshakeCompleted(kSessionCompleted);
        if (ret) {
          tylog(
              "already recv rtp, we can handshakeCompleted safely, but ret=%d, "
              "but not return error",
              ret);
        }
      }
      ret = rtpHandler_.HandleRtpPacket(vBufReceive);
      if (ret) {
        tylog("handle rtp packet fail, ret=%d", ret);
        return ret;
      }
      break;
    }

    default:
      tylog("unknown packet type %s", PacketTypeToString(packType).data());
      std::string echoStr("Hello guy :)");
      this->SendToClient({echoStr.begin(), echoStr.end()});

      if (std::string(vBufReceive.begin(), vBufReceive.end()) == "Hello Qt!" &&
          stateMachine_ < EnumStateMachine::GOT_RTP) {
        this->stateMachine_ = EnumStateMachine::GOT_RTP;
        tylog("set stateMachine to %s.",
              StateMachineToString(stateMachine_).data());
        this->sdpHandler_.bNotUseSrtp = true;
        this->bUseRsfec = true;

        // if pull fail, retry?
        // but current branch is run only once
        const char *url = std::getenv("TY_PULL_URL");
        if (nullptr != url) {
          tylog("pull url=%s", url);

          RtmpPuller &rtmpPuller = *new RtmpPuller(*this);  // FIXME

          ret = pullHandler_.InitPullHandler(
              &rtmpPuller.rtmp_.m_sb.sb_socket,
              std::bind(&RtmpPuller::InitProtocolHandler, &rtmpPuller, url),
              std::bind(&RtmpPuller::HandlePacket, &rtmpPuller),
              std::bind(&RtmpPuller::CloseStream, &rtmpPuller));
          if (ret) {
            tylog("Handler.handshakeTo ret=%d.", ret);

            // return ret;
          }
        } else {
          tylog("pull url env var not exist");
        }
      }

      break;
  }

  return 0;
}

std::string PeerConnection::ToString() const {
  return tylib::format_string(
      "{state=%s, client=%s:%d, rtpHandler=%s, initTime=%s, lastLiveTime=%s}",
      StateMachineToString(stateMachine_).data(), clientIP_.data(), clientPort_,
      rtpHandler_.ToString().data(),
      tylib::MilliSecondToLocalTimeString(initTimeMs_).data(),
      tylib::MilliSecondToLocalTimeString(lastActiveTimeMs_).data());
}

std::shared_ptr<PeerConnection> PeerConnection::FindPeerPC() const {
  std::shared_ptr<PeerConnection> pc;

  for (const auto &p : Singleton<PCManager>::Instance().client2PC_) {
    if (clientIP_ == p.first.ip && clientPort_ == p.first.port) {
      continue;
    }

    // OPT: pc's key should be ice username, and add map(addr=>pc) for quick
    // find.
    if (iceHandler_.iceInfo_.remoteUsername ==
        p.second->iceHandler_.iceInfo_.remoteUsername) {
      continue;
    }

    if (p.second->stateMachine_ < EnumStateMachine::GOT_RTP) {
      continue;
    }

    if (nullptr == pc || pc->lastActiveTimeMs_ < p.second->lastActiveTimeMs_) {
      pc = p.second;
    }
  }

  if (nullptr != pc) {
    tylog("found peer=%s.", pc->ToString().data());
  } else {
    tylog("not found peer");
  }

  return pc;
}
}  // namespace tywebrtc
