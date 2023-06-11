// OPT: not use global thing as possible
// will refactor

#pragma once

#include <sys/stat.h>

#include <cassert>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "prometheus/family.h"
#include "prometheus/gauge.h"

const int kUplossRateMul100 = 0;
const int kDownlossRateMul100 = 0;
const int kPCDeadTimeoutMs = 1 * 1000;

const int kGuessMtuByte = 1200;

extern prometheus::Family<prometheus::Gauge>* g_startServer;
extern prometheus::Family<prometheus::Gauge>* g_recvPacketNum;

extern int g_sock_fd;
extern int g_dumpRecvSockfd;
extern int g_dumpSendSockfd;

const int kDownlinkAudioSsrc = 16854838;  // taylor to make dynamic
const int kDownlinkAudioPayloadType = 111;
const constexpr int kDownlinkVideoSsrc = 33697348;  // taylor to make dynamic
const int kDownlinkH264PayloadType = 106;

class PeerConnection;

// TODO: should save to remote DB ? must refactor! Now we use singleton
// OPT2: move to tylib
template <class T>
class Singleton : private T {
 public:
  // may have param
  static T& Instance() {
    // 1. C++11: If control enters the declaration concurrently while the
    // variable is being initialized, the concurrent execution shall wait for
    // completion of the initialization.
    // 2. Lazy evaluation.
    static Singleton<T> s;

    return s;
  }

  Singleton(const Singleton&) = delete;
  Singleton& operator=(const Singleton&) = delete;

 private:
  Singleton() {}
  ~Singleton() {}
};

// manage all peerConnection, in singleton
class PCManager {
 public:
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

  std::shared_ptr<PeerConnection> GetPeerConnection(int targetFd) const;

  // should be private
 public:
  // don't use global variable STL
  // taylor 定时清理超时会话（pc）
  // std::map is not designed to work with objects which are not
  // copy-constructible. But PeerConnection cannot be copy because its member
  // has reference data member
  // https://stackoverflow.com/questions/20972751/how-to-put-a-class-that-has-deleted-copy-ctor-and-assignment-operator-in-map
  std::map<ClientSrcId, std::shared_ptr<PeerConnection>> client2PC_;
};

void DumpRecvPacket(const std::vector<char>& packet);
void DumpSendPacket(const std::vector<char>& packet);

// tmp
inline int mkdir_p(const char* path, mode_t mode) {
  const char* p;
  p = strchr(path + 1, '/');

  struct stat st;
  while (1) {
    if (!p) {
      int n;
      if ((n = strlen(path)) > 0 && path[n - 1] != '/') {
        if (stat(path, &st) < 0 && errno == ENOENT &&
            (mkdir(path, mode) < 0 || chmod(path, mode) < 0))
          return -1;
      }
      break;
    }

    std::string name = std::string(path, p - path);

    if (stat(name.c_str(), &st) < 0 && errno == ENOENT &&
        (mkdir(name.c_str(), mode) < 0 || chmod(name.c_str(), mode) < 0))
      return -2;

    p = strchr(p + 1, '/');
  }

  return 0;
}