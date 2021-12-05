#ifndef PC_PEER_CONNECTION_H_
#define PC_PEER_CONNECTION_H_

#include <arpa/inet.h>
#include <netinet/in.h>

#include <cstring>

#include "dtls/dtls_handler.h"
#include "ice/ice_handler.h"
#include "log/log.h"
#include "rtp/rtp_handler.h"
#include "rtp/srtp/srtp_handler.h"
#include "sdp/sdp_handler.h"

enum class PacketType {
  STUN,
  DTLS,
  RTP,
  UNKNOWN,
};

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
  if ((cSubCmd == 0) || (cSubCmd == 1)) return PacketType::STUN;

  if ((cSubCmd >= 20) && (cSubCmd <= 64)) return PacketType::DTLS;

  if ((cSubCmd >= 128) && (cSubCmd <= 191)) return PacketType::RTP;

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

  int HandlePacket(const std::vector<char>& vBufReceive);

  void StoreClientIPPort(const std::string& ip, int port) {
    clientIP_ = ip;
    clientPort_ = port;
    tylog("src ip=%s, port=%d", clientIP_.data(), clientPort_);
  }

  // private:
  enum EnumStateMachine stateMachine_;

  SdpHandler sdpHandler_;  // for signal

  IceHandler iceHandler_;
  DtlsHandler dtlsHandler_;
  RtpHandler rtpHandler_;
  SrtpHandler srtpHandler_;

  std::string clientIP_;
  int clientPort_ = 0;
};

#endif  // PC_PEER_CONNECTION_H_
