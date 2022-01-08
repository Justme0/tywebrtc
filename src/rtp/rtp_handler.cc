#include "rtp/rtp_handler.h"

#include <arpa/inet.h>

#include <cstring>
#include <string>

#include "log/log.h"
#include "pc/peer_connection.h"
#include "rtp/rtp_codec.h"

RtpHandler::RtpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

std::string g_anotherServerIp = "192.168.124.13";
extern int g_sock_fd;
extern struct sockaddr_in g_stConnAddr;  // to use data member

int RtpHandler::HandleRtpPacket(const std::vector<char> &vBufReceive) {
  int ret = 0;

  // if we recv web's data, dtls should complete in Chrome
  // OPT: no need call hand shake complete function each time recv rtp
  const bool kSessionCompleted = true;
  ret = belongingPeerConnection_.dtlsHandler_.HandshakeCompleted(
      kSessionCompleted);
  if (ret) {
    tylog(
        "already recv rtp, we can handshakeCompleted safely, but ret=%d, but "
        "not return error",
        ret);
  }

  belongingPeerConnection_.stateMachine_ = EnumStateMachine::GOT_RTP;

  if (RtpRtcpStrategy::isRTCP(vBufReceive)) {
    ret = belongingPeerConnection_.srtpHandler_.UnprotectRtcp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("unprotect RTCP fail, ret=%d", ret);
      return ret;
    }
  } else {
    // reuse original buffer
    // taylor think restart svr
    ret = belongingPeerConnection_.srtpHandler_.UnprotectRtp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("unprotect RTP (not RTCP) fail, ret=%d", ret);
      return ret;
    }
  }

  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(vBufReceive.data());
  tylog("recv rtp=%s", rtpHeader.ToString().data());

  // ssize_t sendtoLen =
  //     sendto(g_sock_fd, vBufReceive.data(), vBufReceive.size(), 0,
  //            reinterpret_cast<struct sockaddr *>(&g_stConnAddr),
  //            sizeof(struct sockaddr_in));
  // if (-1 == sendtoLen) {
  //   tylog("sendto errorno=%d[%s]", errno, strerror(errno));
  //   return -4;
  // }
  // tylog("sendto reply buf size=%ld", sendtoLen);

  return 0;
}
