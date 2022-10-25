#include "pc/peer_connection.h"

#include "log/log.h"
#include "tylib/time/time_util.h"
#include "tylib/time/timer.h"

PeerConnection::PeerConnection()
    : stateMachine_(EnumStateMachine::GOT_CANDIDATE),  // sdp has candiate
      sdpHandler_(*this),
      iceHandler_(*this),
      dtlsHandler_(*this, false),  // taylor 写死 dtls client
      rtpHandler_(*this),
      srtpHandler_(*this),
      initTimeMs_(g_now_ms) {}

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