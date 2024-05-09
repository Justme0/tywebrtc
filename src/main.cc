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

#include "colib/co_routine.h"
#include "prometheus/exposer.h"
#include "prometheus/gauge.h"
#include "prometheus/registry.h"
#include "tylib/ip/ip.h"
#include "tylib/string/format_string.h"
#include "tylib/time/timer.h"

#include "src/global_tmp/global_tmp.h"
#include "src/log/log.h"
#include "src/monitor/monitor.h"
#include "src/pc/peer_connection.h"
#include "src/rtmp/rtmp_pull.h"
#include "timer/timer.h"

using tywebrtc::g_startServer;
using tywebrtc::g_recvPacketNum;
using tywebrtc::g_sock_fd;
using tywebrtc::g_dumpRecvSockfd;
using tywebrtc::g_dumpSendSockfd;
using tywebrtc::g_pRegistry;
using tywebrtc::kUplossRateMul100;

// TODO: use tywebrtc namespace?

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

  tylog("recv from return len=%ld (application layer)", iRecvLen);

  if (iRecvLen < -1) {
    // should not appear
    tylog("unknown errno %d[%s]", errno, strerror(errno));

    return -1;
  } else if (-1 == iRecvLen) {
    tylog("recv from ret=-1, errno %d[%s]", errno, strerror(errno));
    if (EAGAIN == errno) {
      return 0;
    } else {
      return -1;
    }
  } else if (iRecvLen == 0) {
    tylog("peer shutdown (not error)");

    return 0;
  } else if (iRecvLen > (int)(vBufReceive.size())) {
    // } else if (iRecvLen > static_cast<int>(vBufReceive.size())) {
    tylog("recv buffer overflow len=%ld", iRecvLen);

    return -3;
  }
  vBufReceive.resize(iRecvLen);

  std::string ip;
  int port = 0;
  tylib::ParseIpPort(address, ip, port);
  tylog(
      "=================================="
      " src ip=%s, port=%d recv size=%zu "
      "==================================",
      ip.data(), port, vBufReceive.size());

  // get some pc according to clientip, port or ICE username (to FIX),
  // cannot handle ICE connection change
  std::shared_ptr<tywebrtc::PeerConnection> pc =
      tywebrtc::Singleton<tywebrtc::PCManager>::Instance().GetPeerConnection(
          ip, port, "");

  // must before srtp if it's rtp, otherwise srtp_err_status_replay_fail
  // https://segmentfault.com/a/1190000040211375
  int r = rand() % 100;
  if (r < kUplossRateMul100) {
    tylog("up rand=%d lostrate=%d%%, drop!", r, kUplossRateMul100);

    return 0;
  }

  ret = pc->HandlePacket(vBufReceive);
  if (ret) {
    tylog("pc handlePacket fail, ret=%d", ret);
    return ret;
  }

  return 0;
}

// const int kMultiplexIOMaxEventNum = 1024;

// ref:
// https://stackoverflow.com/questions/142508/how-do-i-check-os-with-a-preprocessor-directive
// todo implement for windows and other OS
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

/*
#include <sys/epoll.h>

int g_efd;  // tmp

void CrossPlatformNetworkIO() {
  tylogAndPrintfln("in Linux");
  g_efd = epoll_create(
      kMultiplexIOMaxEventNum);  // if media data IO frequently, use select(2)
  if (g_efd == -1) {
    tylogAndPrintfln("epoll_create return -1, errno=%d[%s]", errno,
                     strerror(errno));

    return;
  }

  epoll_event event{};
  event.data.fd = g_sock_fd;
  // level trigger
  event.events = EPOLLIN | EPOLLHUP | EPOLLERR | EPOLLRDHUP;
  if (epoll_ctl(g_efd, EPOLL_CTL_ADD, g_sock_fd, &event) == -1) {
    tylogAndPrintfln("epoll ctl return -1, add g_sockfd=%d failed errno=%d[%s]",
                     g_sock_fd, errno, strerror(errno));

    return;
  }

  epoll_event events[kMultiplexIOMaxEventNum]{};
  tylogAndPrintfln("to loop");
  while (1) {
    int nfds = epoll_wait(g_efd, events, kMultiplexIOMaxEventNum, -1);
    if (-1 == nfds) {
      tylog("epoll wait return -1, errno=%d[%s]", errno, strerror(errno));

      continue;
    }

    assert(nfds >= 0);

    if (nfds == 0) {
      tylog("epoll nothing happened");

      continue;
    }

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
      } else if (std::shared_ptr<PeerConnection> pc =
                     Singleton<PCManager>::Instance().GetPeerConnection(fd)) {
        // OPT: use epoll data.ptr O(1) avoid the shit p_playSocket_
        tylog("recv rtmp packet from socket fd=%d.", fd);
        if (events[i].events | EPOLLIN) {
          int ret = pc->pullHandler_.HandlePacket();
          if (ret) {
            tylog("handleRequest fail, ret=%d", ret);
          }
        } else {
          tylog("unexpect epoll events=%d", events[i].events);
        }
      } else {
        tylog("NOTE: unknown fd=%d.", fd);

        // may have timer fd
        // uint64_t exp;
        // int s = read(fd, &exp, sizeof(uint64_t));
        // if (-1 == s) {
        //   tylog("read return -1, errno=%d[%s]", errno, strerror(errno));
        //   continue;
        // }
        // if (s != sizeof(uint64_t)) {
        //   tylog("read timerfd failed, read fd=%d, return %d", fd, s);
        // } else {
        //   tylog("shit unknown");
        //   // HandleJitter(fd, exp);
        // }
      }
    }
  }
}
*/

#endif

static void InitTimer() {
  TimerManager::Instance()->AddTimer(new tywebrtc::MonitorStateTimer);
}

#define INIT_LOG_V2(path, format, level, size)                        \
  do {                                                                \
    tywebrtc::mkdir_p(path, 0777);                                    \
    tylib::MLOG_INIT(MLOG_DEF_LOGGER, level, format, path, "", size); \
  } while (0)

int InitDumpSock() {
  int ret = 0;

  // init recv socket
  ret = socket(AF_INET, SOCK_DGRAM, 0);
  if (ret == -1) {
    tylogAndPrintfln("cannot open dump socket, ret=%d, errno=%d[%s]", ret,
                     errno, strerror(errno));
    return ret;
  }
  g_dumpRecvSockfd = ret;

  // bind
  const char* kLocalIP = "127.0.0.1";  // maybe should in config
  int kListenPort = 12346;
  struct sockaddr_in address;
  bzero(&address, sizeof(address));
  address.sin_family = AF_INET;
  inet_pton(AF_INET, kLocalIP, &address.sin_addr);
  address.sin_port = htons(kListenPort);
  tylogAndPrintfln("sendSocket to bind to %s:%d", kLocalIP, kListenPort);

  ret = bind(g_dumpRecvSockfd, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address));
  if (ret == -1) {
    tylogAndPrintfln("bind return -1, errno=%d[%s]", errno, strerror(errno));

    return ret;
  }
  tylogAndPrintfln("bind socket succ");

  // init recv socket
  ret = socket(AF_INET, SOCK_DGRAM, 0);
  if (ret == -1) {
    tylogAndPrintfln("cannot open dump socket, ret=%d, errno=%d[%s]", ret,
                     errno, strerror(errno));
    return ret;
  }
  g_dumpSendSockfd = ret;

  // bind
  kListenPort = 12348;
  bzero(&address, sizeof(address));
  address.sin_family = AF_INET;
  inet_pton(AF_INET, kLocalIP, &address.sin_addr);
  address.sin_port = htons(kListenPort);
  tylogAndPrintfln("sendSocket to bind to %s:%d", kLocalIP, kListenPort);

  ret = bind(g_dumpSendSockfd, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address));
  if (ret == -1) {
    tylogAndPrintfln("bind return -1, errno=%d[%s]", errno, strerror(errno));

    return ret;
  }
  tylogAndPrintfln("bind socket succ");

  return 0;
}

int InitMonitor() {
  // https://github.com/jupp0r/prometheus-cpp
  // create an http server
  const int kMonitorPort = 4444;
  tylogAndPrintfln("bind prometheus http port=%d", kMonitorPort);
  // Exporser object should always alive
  // https://github.com/jupp0r/prometheus-cpp/issues/559#issuecomment-1068933850
  prometheus::Exposer* g_pExposer =
      new prometheus::Exposer{tylib::format_string("0.0.0.0:%d", kMonitorPort)};
  // prometheus::Exposer exposer{{
  // "listening_ports", "127.0.0.1:8091",
  // "num_threads", "2",
  // "enable_keep_alive", "yes",
  // "keep_alive_timeout_ms", "90000", }};

  // create a metrics registry
  // @note it's the users responsibility to keep the object alive
  g_pRegistry = std::make_shared<prometheus::Registry>();

  // case 1
  // add a new counter family to the registry (families combine values with the
  // same name, but distinct label dimensions)
  //
  // @note please follow the metric-naming best-practices:
  // https://prometheus.io/docs/practices/naming/

  // case 2
  // add a counter whose dimensional data is not known at compile time
  // nevertheless dimensional values should only occur in low cardinality:
  // https://prometheus.io/docs/practices/naming/#labels
  g_startServer = &prometheus::BuildGauge()
                       .Name("start_server")
                       .Help("Number of server start")
                       .Register(*g_pRegistry);
  g_recvPacketNum = &prometheus::BuildGauge()
                         .Name("recv_packet")
                         .Help("Number of recv packet")
                         .Register(*g_pRegistry);

  // ask the exposer to scrape the registry on incoming HTTP requests
  g_pExposer->RegisterCollectable(g_pRegistry);

  return 0;
}

void* libcoStart(void*) {
  co_enable_hook_sys();

  for (;;) {
    struct pollfd pf = {0, 0, 0};
    pf.fd = g_sock_fd;
    pf.events = (POLLIN | POLLERR | POLLHUP);
    co_poll(co_get_epoll_ct(), &pf, 1, 10000);

    int ret = HandleRequest();
    if (ret) {
      tylog("handleRequest ret=%d", ret);
    }
  }

  return nullptr;
}

void* mytimer(void*) {
  co_enable_hook_sys();

  for (;;) {
    TimerManager::Instance()->UpdateTimers(Time());
    // sleep 1ms
    poll(NULL, 0, 1);
  }
}

int main() {
  int ret = 0;

  tylogAndPrintfln("nowMs=%ld.", g_now_ms);

  // * init random seed
  srand(g_now_ms + getpid());

  // * init log
  INIT_LOG_V2("./log/",
              tylib::MLOG_F_TIME | tylib::MLOG_F_FILELINE | tylib::MLOG_F_FUNC,
              6, 100 * 1024 * 1024);

  // before server launch, print log to standard out and file

  // * init const dump socket
  ret = InitDumpSock();
  if (ret) {
    tylogAndPrintfln("init dump ret=%d", ret);

    return ret;
  }

  // * init monitor
  ret = InitMonitor();
  if (ret) {
    tylog("init monitor ret=%d.", ret);

    return ret;
  }

  tylogAndPrintfln("OPENSSL_VERSION_NUMBER=%#lx < 0x10100000 is %d",
                   OPENSSL_VERSION_NUMBER,
                   OPENSSL_VERSION_NUMBER < 0x10100000L);

  // step 1: create socket
  ret = socket(PF_INET, SOCK_DGRAM, 0);
  if (ret == -1) {
    tylogAndPrintfln("create listen socket failed, ret=%d, errno=%d[%s]", ret,
                     errno, strerror(errno));
    return ret;
  }
  g_sock_fd = ret;
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

    return ret;
  }
  tylogAndPrintfln("bind succ");

  // udp no listen
  // ret = listen(g_sock_fd, 5);
  // if (-1 == ret) {
  //     tylogAndPrintfln("listen return -1, errno=%d[%s]", errno,
  //     strerror(errno)); return 0;
  // }

  // * init timer
  InitTimer();

  g_startServer->Add({{"dummy", "startServer"}}).Increment();

  // step 3: event loop
  // CrossPlatformNetworkIO();

  {
    ret = tywebrtc::SetNonBlock(g_sock_fd);
    if (ret) {
      tylog("setNonBlock ret=%d.", ret);
      return ret;
    }

    // create recv client req
    stCoRoutine_t* co = nullptr;
    ret = co_create(&co, nullptr, libcoStart, nullptr);
    if (ret) {
      tylog("co create ret=%d.", ret);
      return ret;
    }
    co_resume(co);
  }

  {
    // create timer
    stCoRoutine_t* co = nullptr;
    ret = co_create(&co, nullptr, mytimer, nullptr);
    if (ret) {
      tylog("co create ret=%d.", ret);
      return ret;
    }
    co_resume(co);
  }

  co_eventloop(co_get_epoll_ct(), 0, 0);

  return 0;
}