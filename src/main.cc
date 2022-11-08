#include <arpa/inet.h>
#include <net/if.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tylib/ip/ip.h"
#include "tylib/time/timer.h"

#include "log/log.h"
#include "pc/peer_connection.h"

#include "global_tmp/global_tmp.h"

// TODO: use tywebrtc namespace?

int g_sock_fd;

// Execute `system` and get output, OPT: move to lib
// https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po
std::string execCmd(const char* cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

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

// to move to tylib, now no use
int GetLanIp(std::string* o_ip) {
  // should also provided by env var or cmd option
  const std::string& kIpFile = "conf/LOCAL_IP.txt";
  std::ifstream infile(kIpFile);
  if (!(infile >> *o_ip)) {
    tylog("open %s fail", kIpFile.data());
    return -1;
  }

  if (o_ip->empty()) {
    // should check ip valid
    tylog("empty ip in file %s", kIpFile.data());
    return -2;
  }

  return 0;

  /*
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
  */
}

int HandleRequest() {
  int ret = 0;

  // to use memory pool for so large buffer, UDP is enough, TCP?
  // OPT: media packet should less copy as possible
  const int kSendRecvUdpMaxLength = 4 * 1024;
  std::vector<char> vBufReceive(kSendRecvUdpMaxLength);

  struct sockaddr_in address;
  socklen_t addr_size = sizeof(struct sockaddr_in);

  ssize_t iRecvLen =
      recvfrom(g_sock_fd, vBufReceive.data(), vBufReceive.size(), 0,
               (struct sockaddr*)&address, (socklen_t*)&addr_size);
  tylog("========================= recv len=%ld (application layer) =========================", iRecvLen);
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
  tylib::ParseIpPort(address, ip, port);
  tylog("src ip=%s, port=%d", ip.data(), port);
  // get some pc according to clientip, port or ICE username (taylor FIX)
  std::shared_ptr<PeerConnection> pc = Singleton::Instance().GetPeerConnection(
      ip, port, "");  // have bug, ufrag is ""
  // pc->StoreClientIPPort(ip, port);  // should be in GetPeerConnection()
  // if (ret) {
  //   tylog("pc storeClientIPPort fail, ret=%d", ret);
  //   return ret;
  // }
  ret = pc->HandlePacket(vBufReceive);
  if (ret) {
    tylog("pc handlePacket fail, ret=%d", ret);
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

void CrossPlatformNetworkIO() { tylogAndPrintfln("in Windows, not implement"); }

#elif __APPLE__ || __FreeBSD__

#include <sys/event.h>

void CrossPlatformNetworkIO() {
  int ret = 0;

  tylogAndPrintfln("in BSD series OS (e.g. mac, freeBSD)");
  int kq = kqueue();

  struct kevent evSet;
  EV_SET(&evSet, g_sock_fd, EVFILT_READ, EV_ADD, 0, 0, nullptr);
  assert(-1 != kevent(kq, &evSet, 1, nullptr, 0, nullptr));

  struct kevent evList[kMultiplexIOMaxEventNum];
  tylogAndPrintfln("to loop");
  while (1) {
    // returns number of events
    int eventNumber =
        kevent(kq, nullptr, 0, evList, kMultiplexIOMaxEventNum, nullptr);
    if (-1 == eventNumber) {
      tylog("kevent return -1, errno=%d[%s]", errno, strerror(errno));
      continue;
    }
    tylog("got %d events", eventNumber);

    for (int i = 0; g_now.ComputeNow(),
             TimerManager::Instance()->UpdateTimers(g_now), i < eventNumber;
         i++) {
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
            tylog("handleRequest fail, ret=%d", ret);
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
      //      if (sendfile(junk, fd, offset, &len, nullptr, 0) != 0) {
      //        //            perror("sendfile");
      //        //            tylog("err %d", errno);

      //          if (errno == EAGAIN) {
      //              // schedule to send the rest of the file
      //              EV_SET(&evSet, fd, EVFILT_WRITE, EV_ADD | EV_ONESHOT, 0,
      //              0, (void *)(offset + len)); kevent(kq, &evSet, 1, nullptr,
      //              0, nullptr);
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

    for (int i = 0; g_now.ComputeNow(),
             TimerManager::Instance()->UpdateTimers(g_now), i < nfds;
         i++) {
      int fd = events[i].data.fd;
      if (fd == g_sock_fd) {
        if (events[i].events | EPOLLIN) {
          int ret = HandleRequest();
          if (ret) {
            tylog("handleRequest fail, ret=%d", ret);
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

class MonitorStateTimer : public Timer {
 public:
  MonitorStateTimer() : Timer(2000, -1) {}

 private:
  bool _OnTimer() override {
    Singleton::Instance().CleanTimeoutPeerConnection();
    tylog("client2pc size=%d.", Singleton::Instance().GetPeerConnectionSize());

    return true;
  }
};

static void InitTimer() {
  TimerManager::Instance()->AddTimer(new MonitorStateTimer);
}

int mkdir_p(const char* path, mode_t mode) {
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
      return -1;

    p = strchr(p + 1, '/');
  }

  return 0;
}

#define INIT_LOG_V2(path, format, level, size)                        \
  do {                                                                \
    mkdir_p(path, 0777);                                              \
    tylib::MLOG_INIT(MLOG_DEF_LOGGER, level, format, path, "", size); \
  } while (0)

int main(int argc, char* argv[]) {
  int ret = 0;

  INIT_LOG_V2("./log/",
              tylib::MLOG_F_TIME | tylib::MLOG_F_FILELINE | tylib::MLOG_F_FUNC,
              6, 512 * 1024 * 1024);

  (void)argc;
  (void)argv;
  // before server launch, print log to standard out and file
  tylogAndPrintfln("OPENSSL_VERSION_NUMBER=%#lx < 0x10100000 is %d",
                   OPENSSL_VERSION_NUMBER,
                   OPENSSL_VERSION_NUMBER < 0x10100000L);

  // step 1: create socket
  g_sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (ret < 0) {
    tylogAndPrintfln("create listen socket failed, ret %d", ret);
    return __LINE__;
  }
  // TODO set nonblock

  // step 2: bind
  const char* kInterfaceAny = "0.0.0.0";  // maybe should in config
  const int kListenPort = 8090;

  struct sockaddr_in address;
  bzero(&address, sizeof(address));
  address.sin_family = AF_INET;
  inet_pton(AF_INET, kInterfaceAny, &address.sin_addr);
  address.sin_port = htons(kListenPort);
  tylogAndPrintfln("to bind to %s:%d", kInterfaceAny, kListenPort);

  ret = bind(g_sock_fd, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address));
  if (ret == -1) {
    tylogAndPrintfln("bind return -1, errno=%d[%s]", errno, strerror(errno));

    return 0;
  }
  tylogAndPrintfln("bind succ");

  // udp no listen
  // ret = listen(g_sock_fd, 5);
  // if (-1 == ret) {
  //     tylogAndPrintfln("listen return -1, errno=%d[%s]", errno,
  //     strerror(errno)); return 0;
  // }

  // step 3: init some component
  InitTimer();

  // step 4: event loop
  CrossPlatformNetworkIO();

  return 0;
}
