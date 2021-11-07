#pragma once

#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>

#include "log/log.h"

#include "dtls_handler.h"
#include "ice_handler.h"
#include "rtp_handler.h"

enum class PacketType {
  STUN,
  DTLS,
  RTP,
  UNKNOWN,
};

inline PacketType getPacketType(unsigned char cSubCmd) {
  if ((cSubCmd == 0) || (cSubCmd == 1)) return PacketType::STUN;

  if ((cSubCmd >= 20) && (cSubCmd <= 64)) return PacketType::DTLS;

  if ((cSubCmd >= 128) && (cSubCmd <= 191)) return PacketType::RTP;

  return PacketType::UNKNOWN;
}

enum class EnumStateMachine {
  SDP_DONE,
  GET_CANDIDATE_DONE,
  ICE_USE_CANDIDATE_DONE,
  // EnumStateMachine_ICE_START,
  ICE_DONE,
  // EnumStateMachine_DTLS_START,
  DTLS_DONE,
  // EnumStateMachine_RTP_START,
};

class PeerConnection {
 public:
  PeerConnection();
  enum EnumStateMachine stateMachine_;

  int HandlePacket(const std::vector<char> &vBufReceive);

  int StoreClientIPPort(const sockaddr_in &addr) {
    char str[INET_ADDRSTRLEN];
    const char *s = inet_ntop(AF_INET, &(addr.sin_addr), str, INET_ADDRSTRLEN);
    if (nullptr == s) {
      tylog("inet_ntop return null, errno=%d[%s]", errno, strerror(errno));
      return -1;
    }
    clientIP_ = str;
    clientPort_ = ntohs(addr.sin_port);
    tylog("src ip=%s, port=%d", clientIP_.data(), clientPort_);

    return 0;
  }

  // private:
  ICEHandler iceHandler_;
  DTLSHandler dtlsHandler_;
  RTPHandler rtpHandler_;

  std::string clientIP_;
  int clientPort_ = 0;
};
