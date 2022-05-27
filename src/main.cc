#include <arpa/inet.h>
#include <net/if.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cassert>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "log/log.h"
#include "pc/peer_connection.h"
#include "tylib/ip/ip.h"

int g_sock_fd;

std::string g_localip;
struct sockaddr_in g_stConnAddr;  // ipv4, must reflector taylor

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

  // if construct map's value is expensive
  // https://stackoverflow.com/questions/97050/stdmap-insert-or-stdmap-find
  // here we can also use insert and update, but lower_bound is more general
  std::shared_ptr<PeerConnection> GetPeerConnection(const std::string& ip,
                                                    int port,
                                                    const std::string& ufrag) {
    ClientSrcId clientSrcId{ip, port};
    auto lb = client2PC_.lower_bound(clientSrcId);
    tylog("client2PC_ size=%zu", client2PC_.size());

    if (lb != client2PC_.end() &&
        !(client2PC_.key_comp()(clientSrcId, lb->first))) {
      assert(nullptr != lb->second);
      assert(lb->second->clientIP_ == ip && lb->second->clientPort_ == port);
      return lb->second;
    } else {
      tylog("new pc, ip=%s, port=%d, ufrag=%s", ip.data(), port, ufrag.data());
      // Use lb as a hint to insert, so it can avoid another lookup
      // OPT: ICE未选上的地址也会为它生成PC，可优化为PC池
      auto i = client2PC_.emplace_hint(
          lb, std::make_pair(clientSrcId, std::make_shared<PeerConnection>()));
      assert(nullptr != i->second);
      i->second->StoreClientIPPort(ip, port);
      return i->second;
    }
  }

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

// @brief return get local ip number, maybe <= 0
int GetEthAddrs(char* ips[], int num) {
  struct ifconf ifc;
  struct ifreq ifr[64];
  struct sockaddr_in sa;
  int sock = -1;
  int cnt = 0;
  int i, n;

  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) >= 0) {
    ifc.ifc_len = sizeof(ifr);
    ifc.ifc_buf = (caddr_t)ifr;
    ifc.ifc_req = ifr;

    if (ioctl(sock, SIOCGIFCONF, (char*)&ifc) == 0) {
      n = ifc.ifc_len / sizeof(struct ifreq);
      for (i = 0; i < n && i < num; i++) {
        if (ioctl(sock, SIOCGIFADDR, &ifr[i]) == 0) {
          memcpy(&sa, &ifr[i].ifr_addr, sizeof(sa));
          sprintf(ips[cnt++], "%s", inet_ntoa(sa.sin_addr));
        }
      }
    }
    close(sock);
  }

  return (sock >= 0 && cnt > 0) ? cnt : -1;
}

bool IsLanAddr(const std::string& ip) {
  /*
   * A类  10.0.0.0    - 10.255.255.255
          11.0.0.0/8
          30.0.0.0/8
   * B类  172.16.0.0  - 172.31.255.255
   * C类  192.168.0.0 - 192.168.255.255
   *     100.64.0.0/10 100.64.0.0 - 100.127.255.255
   * 其他：9.0.0.0/8   9.0.0.1 - 9.255.255.255
   * 环回 127.0.0.1
   */
  uint32_t uiHostIP = tylib::stringToHostOrder(ip);

  return (uiHostIP >= 0x0A000000 && uiHostIP <= 0x0AFFFFFF) ||
         (uiHostIP >= 0x0B000000 && uiHostIP <= 0x0BFFFFFF) ||
         (uiHostIP >= 0x1E000000 && uiHostIP <= 0x1EFFFFFF) ||
         (uiHostIP >= 0xAC100000 && uiHostIP <= 0xAC1FFFFF) ||
         (uiHostIP >= 0xC0A80000 && uiHostIP <= 0xC0A8FFFF) ||
         (uiHostIP >= 0x64400000 && uiHostIP <= 0x647FFFFF) ||
         (uiHostIP >= 0x09000000 && uiHostIP <= 0x09FFFFFF) ||
         (uiHostIP == 0x7f000001);
}

// to move to tylib
int GetLanIp(std::string* o_ip) {
  const int kIpNumber = 20;
  char aip[kIpNumber][20];
  char* ips[kIpNumber];

  for (int i = 0; i < kIpNumber; i++) {
    ips[i] = aip[i];
  }

  int num = GetEthAddrs(ips, kIpNumber);
  if (num <= 0) {
    tylogAndPrintfln("get eth addrs fail, num=%d", num);
    return -1;
  }

  for (int i = 0; i < num; ++i) {
    if (strcmp(ips[i], "127.0.0.1") == 0) {
      tylogAndPrintfln("i=%d get local addresses 127.0.0.1, ignore", i);

      continue;
    }
    tylogAndPrintfln("i=%d get local addresses %s", i, ips[i]);

    if (IsLanAddr(ips[i])) {
      *o_ip = ips[i];

      return 0;
    }
  }

  tylogAndPrintfln("not found eth addr, get ip num=%d", num);

  return -2;
}

int HandleRequest() {
  int ret = 0;

  socklen_t addr_size = sizeof(struct sockaddr_in);
  // to use memory pool for so large buffer, UDP is enough, TCP?
  // OPT: media packet should less copy as possible
  const int kSendRecvUdpMaxLength = 4 * 1024;
  std::vector<char> vBufReceive(kSendRecvUdpMaxLength);

  ssize_t iRecvLen =
      recvfrom(g_sock_fd, vBufReceive.data(), vBufReceive.size(), 0,
               (struct sockaddr*)&g_stConnAddr, (socklen_t*)&addr_size);
  tylog("=============== recv len=%ld (application layer)", iRecvLen);
  if (iRecvLen < -1) {
    // should not appear
    tylog("unknown errno %d[%s]", errno, strerror(errno));

    return -1;
  } else if (-1 == iRecvLen) {
    tylog("received invalid packet, errno %d[%s]", errno, strerror(errno));

    return -2;
  } else if (iRecvLen == 0) {
    tylog("peer shutdown (not error)");

    return 1;
  } else if (iRecvLen > static_cast<int>(vBufReceive.size())) {
    tylog("recv buffer overflow len=%ld", iRecvLen);

    return -3;
  }
  vBufReceive.resize(iRecvLen);

  tylog("taylor recv buffer data addr=%p, size=%zu", vBufReceive.data(),
        vBufReceive.size());

  std::string ip;
  int port = 0;
  tylib::GetIpPort(g_stConnAddr, ip, port);
  tylog("src ip=%s, port=%d", ip.data(), port);
  // get some pc according to clientip, port or ICE username (taylor FIX)
  std::shared_ptr<PeerConnection> pc = Singleton::Instance().GetPeerConnection(
      ip, port, "");  // have bug, ufrag is ""
  tylog("get pc done");
  // pc->StoreClientIPPort(ip, port);  // should be in GetPeerConnection()
  // if (ret) {
  //   tylog("pc storeClientIPPort fail, ret=%d", ret);
  //   return ret;
  // }
  ret = pc->HandlePacket(vBufReceive);
  if (ret) {
    tylog("pc.HandlePacket fail, ret=%d", ret);
    return ret;
  }

  return 0;
}

const int kMultiplexIOMaxEventNum = 1024;

// ref:
// https://stackoverflow.com/questions/142508/how-do-i-check-os-with-a-preprocessor-directive
// taylor todo implement for windows and other OS
#if _WIN32
// _WIN32 is also defined for _WIN64

void CrossPlatformNetworkIO() { tylogAndPrintfln("in Windows"); }

#elif __APPLE__ || __FreeBSD__
#include <sys/event.h>

void CrossPlatformNetworkIO() {
  int ret = 0;

  tylogAndPrintfln("in BSD series OS (e.g. mac, freeBSD)");
  int kq = kqueue();

  struct kevent evSet;
  EV_SET(&evSet, g_sock_fd, EVFILT_READ, EV_ADD, 0, 0, NULL);
  assert(-1 != kevent(kq, &evSet, 1, NULL, 0, NULL));

  struct kevent evList[kMultiplexIOMaxEventNum];
  tylogAndPrintfln("to loop");
  while (1) {
    // returns number of events
    int eventNumber =
        kevent(kq, NULL, 0, evList, kMultiplexIOMaxEventNum, NULL);
    if (-1 == eventNumber) {
      tylog("kevent return -1, errno=%d[%s]", errno, strerror(errno));
      continue;
    }
    tylog("got %d events", eventNumber);

    for (int i = 0; i < eventNumber; i++) {
      const struct kevent& activeEvent = evList[i];
      int fd = activeEvent.ident;

      if (activeEvent.flags & EV_EOF) {
        tylog("Disconnect, close fd=%d",
              fd);  // should convert kevent ToString() ?
        ret = close(fd);
        if (ret == -1) {
          tylog("close fd=%d return -1, errno=%d[%s]. continue, handle next fd",
                fd, errno, strerror(errno));
        }
        // Socket is automatically removed from the kq by the kernel.
      } else if (fd == g_sock_fd) {
        if (activeEvent.flags & EV_ERROR) {
          tylog("register fd=%d, i=%d, EV_ERROR flags=%d, error=%s", fd, i,
                activeEvent.flags, strerror(activeEvent.data));
        } else {
          ret = HandleRequest();
          if (ret) {
            tylog("HandleRequest fail, ret=%d", ret);
          }
        }
      }
      // else ?
      //  else if (activeEvent.filter == EVFILT_READ) {
      //      tylog("kqueue fd is EVFILT_READ");

      //  } else if (activeEvent.filter == EVFILT_WRITE) {
      //      tylog("Ok rtmp write more!");

      //      off_t offset = (off_t)activeEvent.udata;
      //      off_t len = 0;//activeEvent.data;
      //      if (sendfile(junk, fd, offset, &len, NULL, 0) != 0) {
      //        //            perror("sendfile");
      //        //            tylog("err %d", errno);

      //          if (errno == EAGAIN) {
      //              // schedule to send the rest of the file
      //              EV_SET(&evSet, fd, EVFILT_WRITE, EV_ADD | EV_ONESHOT, 0,
      //              0, (void *)(offset + len)); kevent(kq, &evSet, 1, NULL, 0,
      //              NULL);
      //          }
      //      }
      //      bytes_written += len;
      //      tylog("wrote %lld bytes, %lld total", len, bytes_written);
      //  }
    }
  }
}

#elif __linux__
#include <sys/epoll.h>

void CrossPlatformNetworkIO() {
  tylogAndPrintfln("in Linux");
  int efd = epoll_create(
      kMultiplexIOMaxEventNum);  // if media data IO frequently, use select(2)
  if (efd == -1) {
    tylogAndPrintfln("epoll_create return -1, errno=%d[%s]", errno,
                     strerror(errno));

    return;
  }

  struct epoll_event event;
  event.data.fd = g_sock_fd;
  event.events = EPOLLIN | EPOLLHUP | EPOLLERR | EPOLLRDHUP;
  if (epoll_ctl(efd, EPOLL_CTL_ADD, g_sock_fd, &event) == -1) {
    tylogAndPrintfln("epoll_ctl return -1, add g_sockfd=%d failed errno=%d[%s]",
                     g_sock_fd, errno, strerror(errno));

    return;
  }

  struct epoll_event events[kMultiplexIOMaxEventNum];

  tylogAndPrintfln("to loop");
  while (1) {
    int timeout_ms = 20;
    int nfds = epoll_wait(efd, &events[0], kMultiplexIOMaxEventNum, timeout_ms);
    if (-1 == nfds) {
      tylog("epoll_wait return -1, errno=%d[%s]", errno, strerror(errno));
      continue;
    }

    // tylog("nfds=%d", nfds);

    for (int i = 0; i < nfds; i++) {
      int fd = events[i].data.fd;
      if (fd == g_sock_fd) {
        if (events[i].events | EPOLLIN) {
          int ret = HandleRequest();
          if (ret) {
            tylog("HandleRequest fail, ret=%d", ret);
          }
        } else {
          tylog("unexpect epoll events=%d", events[i].events);
        }
      } else {
        uint64_t exp;
        int s = read(fd, &exp, sizeof(uint64_t));
        if (-1 == s) {
          tylog("read return -1, errno=%d[%s]", errno, strerror(errno));
          continue;
        }
        if (s != sizeof(uint64_t)) {
          tylog("read timerfd failed, read fd=%d, return %d", fd, s);
        } else {
          tylog("shit unknown");
          // HandleJitter(fd, exp);
        }
      }
    }
  }
}
#endif

int main(int argc, char* argv[]) {
  // before server launch, print log to standard out and file
  tylogAndPrintfln("OPENSSL_VERSION_NUMBER=%#lx < 0x10100000 is %d",
                   OPENSSL_VERSION_NUMBER,
                   OPENSSL_VERSION_NUMBER < 0x10100000L);
  // TODO
  //  CONFIG stConfig = {0};
  //  g_config = &stConfig;
  //
  //  int ret = InitWorker(argc, argv);
  //  if (ret != 0) {
  //    tylogAndPrintfln("Initialize failed, ret %d", ret);
  //    exit(1);
  //  }

  int ret = 0;

  ret = GetLanIp(&g_localip);
  if (ret) {
    tylogAndPrintfln("get lan ip fail, ret=%d", ret);

    return ret;
  }
  tylogAndPrintfln("important: get local ip=%s", g_localip.data());

  // step 1 create socket
  g_sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (ret < 0) {
    tylogAndPrintfln("create listen socket failed, ret %d", ret);
    return __LINE__;
  }
  // TODO set nonblock

  // step 2 bind
  struct sockaddr_in address;
  bzero(&address, sizeof(address));
  address.sin_family = AF_INET;
  inet_pton(AF_INET, g_localip.data(),
            &address.sin_addr);  // taylor to change addr
  const int kListenPort = 8007;
  address.sin_port = htons(kListenPort);
  tylogAndPrintfln("to bind to %s:%d", g_localip.data(), kListenPort);

  ret = bind(g_sock_fd, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address));
  if (ret == -1) {
    tylogAndPrintfln("bind return -1, errno=%d[%s]", errno, strerror(errno));
    // before run success, should also printf to show problem directly
    return 0;
  }
  tylogAndPrintfln("bind succ");

  // udp no listen
  // ret = listen(g_sock_fd, 5);
  // if (-1 == ret) {
  //     tylogAndPrintfln("listen return -1, errno=%d[%s]", errno,
  //     strerror(errno)); return 0;
  // }

  // step 3
  CrossPlatformNetworkIO();

  return 0;
}
