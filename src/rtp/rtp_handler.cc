#include "rtp/rtp_handler.h"

#include <arpa/inet.h>

#include <cstring>
#include <string>

#include "log/log.h"
#include "pc/peer_connection.h"

RtpHandler::RtpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

std::string g_anotherServerIp = "127.0.0.1";
extern int g_sock_fd;
extern struct sockaddr_in g_stConnAddr;  // to use data member

int RtpHandler::HandleRtpPacket(const std::vector<char> &vBufReceive) {
  // if we recv web's data, dtls should complete in Chrome
  belongingPeerConnection_.dtlsHandler_.HandshakeCompleted(
      /*bSessionCompleted*/ true);
  belongingPeerConnection_.stateMachine_ = EnumStateMachine::GOT_RTP;

  // reuse original buffer
  int ret = belongingPeerConnection_.srtpHandler_.UnprotectRtp(
      const_cast<std::vector<char> *>(&vBufReceive));
  if (ret) {
    return ret;
  }

  ssize_t sendtoLen =
      sendto(g_sock_fd, vBufReceive.data(), vBufReceive.size(), 0,
             reinterpret_cast<struct sockaddr *>(&g_stConnAddr),
             sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    return -4;
  }
  tylog("sendto reply buf size=%ld", sendtoLen);

  return 0;
}
