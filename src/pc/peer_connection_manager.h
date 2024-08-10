// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_PC_PEER_CONNECTION_MANAGER_H_
#define SRC_PC_PEER_CONNECTION_MANAGER_H_

#include <unordered_map>

#include "src/pc/peer_connection.h"

namespace tywebrtc {
struct ClientSrcId {
  std::string ip;
  int port{};

  ClientSrcId(const std::string& ip, int port) : ip(ip), port(port) {}

  // https://en.cppreference.com/w/cpp/utility/hash
  // bool operator==(const ClientSrcId&) const = default;  // since C++20
  bool operator==(const ClientSrcId& other) const {
    return ip == other.ip && port == other.port;
  }

  std::string ToString() const {
    return tylib::format_string("{ip=%s, port=%d}", ip.data(), port);
  }
};
}

template <>
struct std::hash<tywebrtc::ClientSrcId> {
  std::size_t operator()(const tywebrtc::ClientSrcId& obj) const noexcept {
    std::size_t h1 = std::hash<std::string>{}(obj.ip);
    std::size_t h2 = std::hash<int>{}(obj.port);
    return h1 ^ (h2 << 1);  // or use boost::hash_combine
  }
};

namespace tywebrtc {

struct PCKey {
  std::string username;
  int64_t lastActiveTimeMs{};  // last receive data time

  PCKey() {}

  PCKey(const std::string& username, int64_t lastActiveTimeMs)
      : username(username), lastActiveTimeMs(lastActiveTimeMs) {}

  std::string ToString() const {
    return tylib::format_string(
        "{username=%s, lastActiveTimeMs=%s}", username.data(),
        tylib::MilliSecondToLocalTimeString(lastActiveTimeMs).data());
  }
};

// manage all peerConnection, in singleton
class PCManager {
 public:
  int GetPeerConnectionSize() const { return pc_map_.size(); }
  int GetClientAddrSize() const { return addr2username_.size(); }

  void CleanTimeoutPeerConnection();

  PeerConnection* FindOnePeerPC() const;

  PeerConnection* GetPeerConnection(const std::string& ip, int port,
                                    const std::vector<char>& vBufReceive);

  // should be private
 public:
  // don't use global variable STL
  // taylor 定时清理超时会话（pc）
  // std::map is not designed to work with objects which are not
  // copy-constructible. But PeerConnection cannot be copy because its member
  // has reference data member
  // https://stackoverflow.com/questions/20972751/how-to-put-a-class-that-has-deleted-copy-ctor-and-assignment-operator-in-map
  // STUN username => pc
  std::unordered_map<std::string, PeerConnection> pc_map_;
  std::unordered_map<ClientSrcId, PCKey> addr2username_;
};

}  // namespace tywebrtc

#endif  // SRC_PC_PEER_CONNECTION_MANAGER_H_
