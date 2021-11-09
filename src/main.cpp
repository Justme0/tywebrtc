#include <string>
#include <vector>

#include <cassert>
#include <cstdio>
#include <cstring>

#include <arpa/inet.h>
#include <strings.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "log/log.h"
#include "peer_connection.h"

int g_sock_fd;

struct sockaddr_in g_stConnAddr;                  // ipv4
const std::string &g_localip = "192.168.124.13";  // taylor change

class PCIns {
 public:
  static PCIns &ins() {
    static PCIns ins;
    return ins;
  }
  PeerConnection pc;
};

int HandleRequest() {
  int ret = 0;

  socklen_t addr_size = sizeof(struct sockaddr_in);
  std::vector<char> vBufReceive(
      8 * 1024);  // to use memory pool for so large buffer

  ssize_t iRecvLen =
      recvfrom(g_sock_fd, &vBufReceive[0], vBufReceive.size(), 0,
               (struct sockaddr *)&g_stConnAddr, (socklen_t *)&addr_size);
  tylog("recv len=%ld", iRecvLen);
  if (iRecvLen < -1) {
    // should not appear
    tylog("unknown errno %d[%s]\n", errno, strerror(errno));

    return -1;
  } else if (-1 == iRecvLen) {
    tylog("received invalid packet, errno %d[%s]\n", errno, strerror(errno));

    return -2;
  } else if (iRecvLen == 0) {
    tylog("peer shutdown (not error)\n");

    return 1;
  } else if (iRecvLen > vBufReceive.size()) {
    tylog("recv buffer overflow len=%ld", iRecvLen);

    return -3;
  }
  vBufReceive.resize(iRecvLen);

  tylog("taylor recv buffer data addr=%p, size=%zu", vBufReceive.data(),
        vBufReceive.size());

  // get some pc according to clientip, port or ICE username
  PeerConnection &pc = PCIns::ins().pc;
  ret = pc.StoreClientIPPort(g_stConnAddr);
  if (ret) {
    tylog("pc storeClientIPPort fail, ret=%d", ret);
    return ret;
  }
  pc.stateMachine_ = EnumStateMachine::GET_CANDIDATE_DONE;  // taylor TODO
  ret = pc.HandlePacket(vBufReceive);
  if (ret) {
    tylog("pc.HandlePacket fail, ret=%d\n", ret);
    return ret;
  }

  return 0;
}

int main(int argc, char *argv[]) {
  tylog("OPENSSL_VERSION_NUMBER=0x%x, < 0x10100000L is %d",
        OPENSSL_VERSION_NUMBER, OPENSSL_VERSION_NUMBER < 0x10100000L);
  // TODO
  //  CONFIG stConfig = {0};
  //  g_config = &stConfig;
  //
  //  int ret = InitWorker(argc, argv);
  //  if (ret != 0) {
  //    tylog("Initialize failed, ret %d", ret);
  //    exit(1);
  //  }

  int ret = 0;

  // step 1
  g_sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (ret < 0) {
    tylog("create listen socket failed, ret %d", ret);
    return __LINE__;
  }
  // TODO set nonblock

  // step 2
  struct sockaddr_in address;
  bzero(&address, sizeof(address));
  address.sin_family = AF_INET;
  inet_pton(AF_INET, g_localip.data(),
            &address.sin_addr);  // taylor to change addr
  const int kListenPort = 8000;
  address.sin_port = htons(kListenPort);
  tylog("to bind to %s:%d", g_localip.data(), kListenPort);

  ret = bind(g_sock_fd, reinterpret_cast<const sockaddr *>(&address),
             sizeof(address));
  if (ret == -1) {
    tylog("bind return -1, errno=%d[%s]", errno, strerror(errno));
    // before run success, should also printf to show problem directly
    return 0;
  }
  tylog("bind succ");

  // udp no listen
  // ret = listen(g_sock_fd, 5);
  // if (-1 == ret) {
  //     tylog("listen return -1, errno=%d[%s]", errno, strerror(errno));
  //     return 0;
  // }

  // step 3
  int g_efd = epoll_create(1024);  // if media data IO frequently, use select(2)
  if (g_efd == -1) {
    tylog("epoll_create return -1, errno=%d[%s]", errno, strerror(errno));
    return 0;
  }

  struct epoll_event event = {0};
  event.data.fd = g_sock_fd;
  event.events = EPOLLIN | EPOLLHUP | EPOLLERR | EPOLLRDHUP;
  if (epoll_ctl(g_efd, EPOLL_CTL_ADD, g_sock_fd, &event) == -1) {
    tylog("epoll_ctl return -1, add g_sockfd=%d failed errno=%d[%s]", g_sock_fd,
          errno, strerror(errno));
    return 0;
  }

  struct epoll_event events[1024];

  tylog("to loop");
  while (1) {
    int timeout_ms = 20;
    int nfds = epoll_wait(g_efd, &events[0], sizeof(events) / sizeof(events[0]),
                          timeout_ms);
    if (-1 == nfds) {
      tylog("epoll_wait return -1, errno=%d[%s]\n", errno, strerror(errno));
      continue;
    }

    // tylog("nfds=%d", nfds);

    for (int i = 0; i < nfds; i++) {
      int fd = events[i].data.fd;
      if (fd == g_sock_fd) {
        if (events[i].events | EPOLLIN) {
          ret = HandleRequest();
          if (ret) {
            tylog("HandleRequest fail, ret=%d\n", ret);
          }
        } else {
          tylog("unexpect epoll events=%d\n", events[i].events);
        }
      } else {
        uint64_t exp;
        int s = read(fd, &exp, sizeof(uint64_t));
        if (-1 == s) {
          tylog("read return -1, errno=%d[%s]\n", errno, strerror(errno));
          continue;
        }
        if (s != sizeof(uint64_t)) {
          tylog("read timerfd failed, read fd=%d, return %d\n", fd, s);
        } else {
          tylog("shit unknown\n");
          // HandleJitter(fd, exp);
        }
      }
    }
  }

  return 0;
}
