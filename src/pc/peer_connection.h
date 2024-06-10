// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_PC_PEER_CONNECTION_H_
#define SRC_PC_PEER_CONNECTION_H_

#include <arpa/inet.h>
#include <netinet/in.h>

#include <cstdint>
#include <cstring>

#include "src/data_channel/data_channel_handler.h"
#include "src/dtls/dtls_handler.h"
#include "src/ice/ice_handler.h"
#include "src/log/log.h"
#include "src/rtp/rtcp/rtcp_handler.h"
#include "src/rtp/rtp_handler.h"
#include "src/rtp/srtp/srtp_handler.h"
#include "src/sdp/sdp_handler.h"
#include "src/signal/signal_handler.h"
#include "src/timer/timer.h"

namespace tywebrtc {

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
  SDP_DONE,       // now no use
  GOT_CANDIDATE,  // now in initialization
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

// why not use function: we want log line number to see where to set state
#define SET_PC_STATE(pc, newState)                       \
  tylog("set stateMachine %s->%s.",                      \
        StateMachineToString((pc).stateMachine_).data(), \
        StateMachineToString(newState).data());          \
  (pc).stateMachine_ = newState

class PeerConnection {
 public:
  PeerConnection(const std::string& ip, int port);

  PeerConnection(const PeerConnection&) = delete;
  void operator=(const PeerConnection&) = delete;

  int HandlePacket(const std::vector<char>& vBufReceive, const std::string& ip,
                   int port);

  int SendToClient(const std::vector<char>& vBufSend) const;
  int SendToAddr(const std::vector<char>& vBufSend, const std::string& ip,
                 int port) const;

  const std::string& clientIP() const { return clientIP_; }

  int clientPort() const { return clientPort_; }

  void SetClientIPPort(const std::string& ip, int port) {
    tylog("update addr %s:%d -> %s:%d.", clientIP_.data(), clientPort_,
          ip.data(), port);
    clientIP_ = ip;
    clientPort_ = port;
  }

  PeerConnection* FindPeerPC() const;

  std::string ToString() const;

 private:
  // first set in initialize-list, the following member use them.
  // OPT: edge style is bad
  std::string clientIP_;
  int clientPort_{};

 public:  // tmp
  enum EnumStateMachine stateMachine_;

  SdpHandler sdpHandler_;  // for signal

  IceHandler iceHandler_;
  DtlsHandler dtlsHandler_;
  RtpHandler rtpHandler_;
  RtcpHandler rtcpHandler_;
  SrtpHandler srtpHandler_;
  DataChannelHandler dataChannelHandler_;
  SignalHandler signalHandler_;

  PushHandler pushHandler_;
  PullHandler pullHandler_;

  int64_t initTimeMs_ = 0;        // construct *this obj time
  int64_t lastActiveTimeMs_ = 0;  // last receive data time
};

}  // namespace tywebrtc

#endif  // SRC_PC_PEER_CONNECTION_H_
