#ifndef PC_PEER_CONNECTION_H_
#define PC_PEER_CONNECTION_H_

#include <arpa/inet.h>
#include <netinet/in.h>

#include <cstdint>
#include <cstring>

#include "data_channel/data_channel_handler.h"
#include "dtls/dtls_handler.h"
#include "ice/ice_handler.h"
#include "log/log.h"
#include "rtp/rtcp/rtcp_handler.h"
#include "rtp/rtp_handler.h"
#include "rtp/srtp/srtp_handler.h"
#include "sdp/sdp_handler.h"
#include "timer/timer.h"

enum class PacketType {
  STUN,
  DTLS,
  RTP,
  UNKNOWN,
};

// OPT: return string_view
// e.g.
// https://source.chromium.org/chromium/chromium/src/+/main:third_party/webrtc/media/base/rtp_utils.cc;drc=184005f8792002e29052d653f4846121ee7d1f9a;l=163
inline std::string PacketTypeToString(PacketType p) {
  switch (p) {
    case PacketType::STUN:
      return "STUN";
    case PacketType::DTLS:
      return "DTLS";
    case PacketType::RTP:
      return "RTP";
    case PacketType::UNKNOWN:
      return "UNKNOWN";
    default:
      return "UnknownValue[" + std::to_string(static_cast<int>(p)) + "]";
  }
}

inline PacketType getPacketType(uint8_t cSubCmd) {
  if (cSubCmd == 0 || cSubCmd == 1) {
    return PacketType::STUN;
  }

  if (cSubCmd >= 20 && cSubCmd <= 64) {
    return PacketType::DTLS;
  }

  if (cSubCmd >= 128 && cSubCmd <= 191) {
    // RTP/RTCP version is 2 (left two bit), so first char is
    // 1000 0000 ~ 1011 1111
    return PacketType::RTP;
  }

  return PacketType::UNKNOWN;
}

// taylor refactor name
enum class EnumStateMachine {
  SDP_DONE,
  GOT_CANDIDATE,
  GOT_FIRST_ICE,
  GOT_USE_CANDIDATE_ICE,
  DTLS_DONE,
  GOT_RTP,
};

inline std::string StateMachineToString(EnumStateMachine s) {
  switch (s) {
    case EnumStateMachine::SDP_DONE:
      return "SDP_DONE";
    case EnumStateMachine::GOT_CANDIDATE:
      return "GOT_CANDIDATE";
    case EnumStateMachine::GOT_FIRST_ICE:
      return "GOT_FIRST_ICE";
    case EnumStateMachine::GOT_USE_CANDIDATE_ICE:
      return "GOT_USE_CANDIDATE_ICE";
    case EnumStateMachine::DTLS_DONE:
      return "DTLS_DONE";
    case EnumStateMachine::GOT_RTP:
      return "GOT_RTP";
    default:
      return "UnknownValue[" + std::to_string(static_cast<int>(s)) + "]";
  }
}

class PeerConnection {
 public:
  PeerConnection();
  ~PeerConnection();

  int HandlePacket(const std::vector<char>& vBufReceive);

  int SendToClient(const std::vector<char>& vBufSend) const;

  void StoreClientIPPort(const std::string& ip, int port) {
    clientIP_ = ip;
    clientPort_ = port;
    tylog("src ip=%s, port=%d", clientIP_.data(), clientPort_);
  }

  std::shared_ptr<PeerConnection> FindPeerPC() const;

  std::string ToString() const;

  // private:
  enum EnumStateMachine stateMachine_;

  SdpHandler sdpHandler_;  // for signal

  IceHandler iceHandler_;
  DtlsHandler dtlsHandler_;
  RtpHandler rtpHandler_;
  RtcpHandler rtcpHandler_;
  SrtpHandler srtpHandler_;
  DataChannelHandler dataChannelHandler_;

  std::string clientIP_;
  int clientPort_ = 0;

  int64_t initTimeMs_ = 0;        // construct *this obj time
  int64_t lastActiveTimeMs_ = 0;  // last receive data time

  PLITimer pliTimer_;
  DTLSTimer dtlsTimer_;
};

#endif  // PC_PEER_CONNECTION_H_
