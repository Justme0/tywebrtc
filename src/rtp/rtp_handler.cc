#include "rtp/rtp_handler.h"

#include <arpa/inet.h>

#include <cassert>
#include <cstring>
#include <string>

#include "log/log.h"
#include "pc/peer_connection.h"
#include "rtp/rtp_parser.h"

RtpHandler::RtpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

extern int g_sock_fd;
extern struct sockaddr_in g_stConnAddr;  // to use data member

// move to tylib
// static void printAscii(const std::string &s) {
//   char arr[9000]{};
//   char *p = arr;
//
//   for (char c : s) {
//     p += sprintf(p, "%X ", c);
//   }
//   tylog("%s", arr);
// }

int RtpHandler::HandleRtpPacket(const std::vector<char> &vBufReceive) {
  int ret = 0;

  // if we recv web's data, dtls should complete in Chrome
  // OPT: no need call hand shake complete function each time recv rtp
  const bool kSessionCompleted = true;
  // taylor no need call every time?
  ret = belongingPeerConnection_.dtlsHandler_.HandshakeCompleted(
      kSessionCompleted);
  if (ret) {
    tylog(
        "already recv rtp, we can handshakeCompleted safely, but ret=%d, but "
        "not return error",
        ret);
  }

  belongingPeerConnection_.stateMachine_ = EnumStateMachine::GOT_RTP;
  std::string mediaType;
  ret = RtpRtcpStrategy::GetMediaType(vBufReceive, &mediaType);
  if (ret) {
    tylog("get media type fail, ret=%d", ret);
    return ret;
  }

  tylog("receive %s", mediaType.data());

  if (mediaType == kMediaTypeRtcp) {
    ret = belongingPeerConnection_.srtpHandler_.UnprotectRtcp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("unprotect RTCP fail, ret=%d", ret);
      return ret;
    }
  } else if (mediaType == kMediaTypeAudio || mediaType == kMediaTypeVideo) {
    // reuse original buffer
    // taylor think restart svr

    std::string test;
    std::string test2;
    test.append(vBufReceive.begin(), vBufReceive.end());

    ret = belongingPeerConnection_.srtpHandler_.UnprotectRtp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("unprotect RTP (not RTCP) fail, ret=%d", ret);
      return ret;
    }

    test2.append(vBufReceive.begin(), vBufReceive.end());
    // printAscii(test);
    // printAscii(test2);
    if (test != test2) {
      // tylog("is not same");
    } else {
      // tylog("is same");
    }

    const RtpHeader &rtpHeader =
        *reinterpret_cast<const RtpHeader *>(vBufReceive.data());
    tylog("recv rtp=%s", rtpHeader.ToString().data());

    // downlink

    RtpHeader &downlinkRtpHeader = const_cast<RtpHeader &>(rtpHeader);

    if (mediaType == kMediaTypeAudio) {
      const int kDownlinkAudioSsrc = 16854838;  // taylor to make dynamic
      downlinkRtpHeader.setSSRC(kDownlinkAudioSsrc);

      const int kDownlinkAudioPayloadType = 111;
      downlinkRtpHeader.setPayloadType(kDownlinkAudioPayloadType);
    } else {
      const int kDownlinkVideoSsrc = 33697348;  // taylor to make dynamic
      downlinkRtpHeader.setSSRC(kDownlinkVideoSsrc);

      const int kDownlinkVideoPayloadType = 125;  // H.264
      downlinkRtpHeader.setPayloadType(kDownlinkVideoPayloadType);
    }

    ret = this->belongingPeerConnection_.srtpHandler_.ProtectRtp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("downlink protect rtp ret=%d", ret);
      return ret;
    }

    ssize_t sendtoLen =
        sendto(g_sock_fd, vBufReceive.data(), vBufReceive.size(), 0,
               reinterpret_cast<struct sockaddr *>(&g_stConnAddr),
               sizeof(struct sockaddr_in));
    if (-1 == sendtoLen) {
      tylog("sendto errorno=%d[%s]", errno, strerror(errno));
      return -1;
    }
    tylog("sendto reply buf size=%ld", sendtoLen);

  } else {
    tylog("receive unknown type of data=%s, return", mediaType.data());
    assert(!"receive unknown media type, should already filter and return");

    return -2;
  }

  return 0;
}
