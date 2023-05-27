#include "pc/peer_connection.h"

#include "log/log.h"
#include "tylib/ip/ip.h"
#include "tylib/time/time_util.h"
#include "tylib/time/timer.h"

PeerConnection::PeerConnection()
    : stateMachine_(EnumStateMachine::GOT_CANDIDATE),  // sdp has candiate
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

// vBufSend is encrypted data
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
  tylog("subcmd=%hhu, packType=%s", cSubCmd,
        PacketTypeToString(packType).data());
  tylog("stateMachine=%s", StateMachineToString(stateMachine_).data());

  // packType number is little, so we don't use map-callback style, just
  // switch-case
  switch (packType) {
    case PacketType::STUN: {
      ret = iceHandler_.HandleIcePacket(vBufReceive);
      if (ret) {
        tylog("handle ice packet fail, ret=%d", ret);
        return ret;
      }
      break;
    }

    case PacketType::DTLS: {
      ret = dtlsHandler_.HandleDtlsPacket(vBufReceive);
      if (ret) {
        tylog("handle dtls packet fail, ret=%d", ret);
        return ret;
      }
      break;
    }

    case PacketType::RTP: {
      ret = rtpHandler_.HandleRtpPacket(vBufReceive);
      if (ret) {
        tylog("handle rtp packet fail, ret=%d", ret);
        return ret;
      }
      break;
    }

    default:
      tylog("unknown packet type %s", PacketTypeToString(packType).data());
      break;
  }

  return 0;
}

std::string PeerConnection::ToString() const {
  return tylib::format_string(
      "{state=%s, client=%s:%d, initTime=%s, lastLiveTime=%s}",
      StateMachineToString(stateMachine_).data(), clientIP_.data(), clientPort_,
      tylib::MilliSecondToLocalTimeString(initTimeMs_).data(),
      tylib::MilliSecondToLocalTimeString(lastActiveTimeMs_).data());
}

std::shared_ptr<PeerConnection> PeerConnection::FindPeerPC() const {
  for (const auto &p : Singleton<PCManager>::Instance().client2PC_) {
    if (clientIP_ == p.first.ip && clientPort_ == p.first.port) {
      continue;
    }

    if (p.second->stateMachine_ < EnumStateMachine::GOT_RTP) {
      continue;
    }

    tylog("found peer=%s.", p.second->ToString().data());
    return p.second;
  }

  tylog("not found peer");
  return nullptr;
}