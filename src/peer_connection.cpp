#include "peer_connection.h"

#include "log/log.h"

PeerConnection::PeerConnection()
    : stateMachine_(EnumStateMachine::GOT_CANDIDATE),  // sdp has candiate
      iceHandler_(*this),
      dtlsHandler_(*this, false),  // taylor 写死 dtls client
      rtpHandler_(*this),
      srtpHandler_(*this) {}

int PeerConnection::HandlePacket(const std::vector<char> &vBufReceive) {
  int ret = 0;

  uint8_t cSubCmd = vBufReceive.front();
  PacketType packType = getPacketType(cSubCmd);
  tylog("subcmd=%hhu, packType=%d", cSubCmd, packType);
  tylog("stateMachine=%d", stateMachine_);

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
      tylog("unknown packet type %d", packType);
      break;
  }

  return 0;
}
