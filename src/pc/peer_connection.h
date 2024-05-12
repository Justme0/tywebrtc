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

class PeerConnection {
 public:
  PeerConnection(const std::string& ip, int port);
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

  std::string clientIP_;
  int clientPort_ = 0;

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

  SenderReportTimer senderReportTimer_;
  ReceiverReportTimer receiverReportTimer_;
  PLITimer pliTimer_;
  DTLSTimer dtlsTimer_;

  bool bNotUseSrtp = false;
  bool bUseRsfec = false;
};

}  // namespace tywebrtc

#endif  // SRC_PC_PEER_CONNECTION_H_
