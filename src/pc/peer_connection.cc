#include "pc/peer_connection.h"

#include "log/log.h"
#include "tylib/ip/ip.h"
#include "tylib/time/time_util.h"
#include "tylib/time/timer.h"

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
      pullHandler_(*this),
      initTimeMs_(g_now_ms),
      pliTimer_(*this),
      dtlsTimer_(*this) {}

PeerConnection::~PeerConnection() {
  TimerManager::Instance()->KillTimer(&this->pliTimer_);

  // OPT: when dtls completes kill timer
  TimerManager::Instance()->KillTimer(&this->dtlsTimer_);
}

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
  tylog("sendto succ buf size=%ld, ip=%s, port=%d.", sendtoLen,
        clientIP_.data(), clientPort_);

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
      // dump recv data after decrypting according to RTP/RTCP
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
          stateMachine_ != EnumStateMachine::GOT_RTP) {
        this->stateMachine_ = EnumStateMachine::GOT_RTP;
        this->bNotUseSrtp = true;

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

    if (p.second->stateMachine_ < EnumStateMachine::GOT_RTP) {
      continue;
    }

    if (nullptr == pc || pc->initTimeMs_ < p.second->initTimeMs_) {
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