// OPT: not use global thing as possible
// will refactor

#pragma once

#include <cassert>
#include <map>
#include <memory>
#include <string>

class PeerConnection;

// TODO: should save to remote DB ? must refactor! Now we use singleton
// OPT2: move to tylib
class Singleton {
 public:
  static Singleton& Instance() {
    // 1. C++11: If control enters the declaration concurrently while the
    // variable is being initialized, the concurrent execution shall wait for
    // completion of the initialization.
    // 2. Lazy evaluation.
    static Singleton s;

    return s;
  }

  struct ClientSrcId {
    std::string ip;
    int port;
    ClientSrcId(const std::string& ip, int port) : ip(ip), port(port) {}

    bool operator<(const ClientSrcId& that) const {
      return std::tie(ip, port) < std::tie(that.ip, that.port);
    }
  };

  int GetPeerConnectionSize() const { return client2PC_.size(); }

  void CleanTimeoutPeerConnection();

  std::shared_ptr<PeerConnection> GetPeerConnection(const std::string& ip,
                                                    int port,
                                                    const std::string& ufrag);

 private:
  Singleton(const Singleton&) = delete;
  Singleton& operator=(const Singleton&) = delete;

  Singleton() {}

  // don't use global variable STL
  // taylor 定时清理超时会话（pc）
  // std::map is not designed to work with objects which are not
  // copy-constructible. But PeerConnection cannot be copy because its member
  // has reference data member
  // https://stackoverflow.com/questions/20972751/how-to-put-a-class-that-has-deleted-copy-ctor-and-assignment-operator-in-map
  std::map<ClientSrcId, std::shared_ptr<PeerConnection>> client2PC_;
};
