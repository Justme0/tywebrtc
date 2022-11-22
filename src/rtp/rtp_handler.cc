#include "rtp/rtp_handler.h"

#include <arpa/inet.h>

#include <cassert>
#include <cstring>
#include <string>

#include "tylib/ip/ip.h"
#include "tylib/string/any_to_string.h"

#include "global_tmp/global_tmp.h"
#include "log/log.h"
#include "pc/peer_connection.h"
#include "rtp/pack_unpack/rtp_to_h264.h"
#include "rtp/rtcp/rtcp_parser.h"
#include "rtp/rtp_parser.h"

extern int g_dumpsock_fd;

// string enum, for print convenience
const std::string kMediaTypeRtcp = "rtcp";
const std::string kMediaTypeVideo = "video";
const std::string kMediaTypeAudio = "audio";

RtpHandler::RtpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

extern int g_sock_fd;

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

int DumpPacketH264(const std::vector<char> &packet,
                   H264Unpacketizer &unpacker) {
  return 0;
  int ret = 0;
  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(packet.data());

  std::string mediaType = rtpHeader.GetMediaType();

  // if audio, to dump OPUS file
  if (mediaType == kMediaTypeVideo) {
    std::vector<MediaData> media;
    ret = unpacker.Unpacketize(packet, &media);
    if (ret) {
      tylog("unpacketize rtp ret=%d", ret);
      return ret;
    }

    tylog("unpack media.size=%zu", media.size());
    for (const MediaData &m : media) {
      ret = unpacker.DumpRawStream(m.data_, rtpHeader.getSSRC());
      if (ret) {
        tylog("dump ret=%d", ret);
        return ret;
      }
    }
  }

  return 0;
}

// int RtpHandler::Release

SSRCInfo::SSRCInfo() : rtpReceiver(*this), rtpSender(*this) {}

int RtpHandler::SendToPeer_(std::vector<char> &packet) {
  int ret = 0;
  RtpHeader &downlinkRtpHeader = *reinterpret_cast<RtpHeader *>(packet.data());

  std::string mediaType = downlinkRtpHeader.GetMediaType();
  tylog("downlink send media type=%s.", mediaType.data());

  if (mediaType == kMediaTypeAudio) {
    // taylor audio not use pacing
    downlinkRtpHeader.setSSRC(kDownlinkAudioSsrc);

    downlinkRtpHeader.setPayloadType(kDownlinkAudioPayloadType);
  } else if (mediaType == kMediaTypeVideo) {
    downlinkRtpHeader.setSSRC(kDownlinkVideoSsrc);

    downlinkRtpHeader.setPayloadType(kDownlinkVideoPayloadType);
  } else {
    tylog("invalid downlink send media type=%s.", mediaType.data());
    assert(!"invalid media type");
  }

  ret = this->belongingPeerConnection_.srtpHandler_.ProtectRtp(
      const_cast<std::vector<char> *>(&packet));
  if (ret) {
    tylog("downlink protect rtp ret=%d", ret);
    return ret;
  }

  sockaddr_in addr =
      tylib::ConstructSockAddr(this->belongingPeerConnection_.clientIP_,
                               this->belongingPeerConnection_.clientPort_);
  ssize_t sendtoLen =
      sendto(g_sock_fd, packet.data(), packet.size(), 0,
             reinterpret_cast<sockaddr *>(&addr), sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    return -1;
  }
  tylog("sendto reply succ buf size=%ld, ip=%s, port=%d.", sendtoLen,
        belongingPeerConnection_.clientIP_.data(),
        belongingPeerConnection_.clientPort_);

  return 0;
}

void DumpPacket(const std::vector<char> &packet) {
  sockaddr_in addr = tylib::ConstructSockAddr("127.0.0.1", 12347);
  ssize_t sendtoLen =
      sendto(g_dumpsock_fd, packet.data(), packet.size(), 0,
             reinterpret_cast<sockaddr *>(&addr), sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    // return -1;
  }
}

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
  std::string mediaType =
      reinterpret_cast<const RtpHeader *>(vBufReceive.data())->GetMediaType();
  tylog("receive %s", mediaType.data());

  // should refactor if else for media type

  if (mediaType == kMediaTypeRtcp) {
    ret = belongingPeerConnection_.srtpHandler_.UnprotectRtcp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("unprotect RTCP fail, ret=%d", ret);

      return ret;
    }

    DumpPacket(vBufReceive);

    ret = belongingPeerConnection_.rtcpHandler_.HandleRtcpPacket(vBufReceive);
    if (ret) {
      tylog("handleRtcpPacket fail, ret=%d", ret);

      return ret;
    }
  } else if (mediaType == kMediaTypeAudio || mediaType == kMediaTypeVideo) {
    // test video loss,
    // must before srtp, otherwise srtp_err_status_replay_fail
    // https://segmentfault.com/a/1190000040211375
    if (mediaType == kMediaTypeVideo) {
      const int kUplossRate = 100;
      int dropKey = rand() % 1000;

      if (dropKey < kUplossRate) {
        tylog("up dropKey=%d lostrate=%d%%, drop! rtp=%s.", dropKey,
              kUplossRate,
              reinterpret_cast<const RtpHeader *>(vBufReceive.data())
                  ->ToString()
                  .data());

        return 0;
      }
    }

    tylog("before unprotect, paddinglen=%d", getRtpPaddingLength(vBufReceive));
    // reuse original buffer
    // taylor consider restart svr
    ret = belongingPeerConnection_.srtpHandler_.UnprotectRtp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("unprotect RTP (not RTCP) fail ret=%d", ret);
      return ret;
    }
    tylog("after unprotect, paddinglen=%d", getRtpPaddingLength(vBufReceive));

    const RtpHeader &rtpHeader =
        *reinterpret_cast<const RtpHeader *>(vBufReceive.data());
    tylog("recv rtp=%s.", rtpHeader.ToString().data());

    if (mediaType == kMediaTypeAudio) {
      g_UplinkAudioSsrc = rtpHeader.getSSRC();
    } else if (mediaType == kMediaTypeVideo) {
      g_UplinkVideoSsrc = rtpHeader.getSSRC();
    }

    DumpPacket(vBufReceive);

    SSRCInfo &ssrcInfo = this->ssrcInfoMap_[rtpHeader.getSSRC()];

    const uint16_t itemSeq = rtpHeader.getSeqNumber();
    int64_t itemCycle = 0;

    // monitor corner case 12 packet
    if (itemSeq >= 65530 || itemSeq <= 5) {
      tylog("monitor corner case, packet=%s.", rtpHeader.ToString().data());
    }

    // update 3 var: ssrcInfo.biggestSeq, ssrcInfo.biggestCycle, itemCycle
    if (itemSeq == ssrcInfo.biggestSeq) {
      // todo more logic
      tylog("recv repeated rtp packet, ignoer it=%s.",
            rtpHeader.ToString().data());
      itemCycle = ssrcInfo.biggestCycle;
    } else if (itemSeq > ssrcInfo.biggestSeq) {
      if (AheadOf(itemSeq, ssrcInfo.biggestSeq)) {
        // case 1 recv newer, not rollback, most common case
        // ssrcInfo.biggestCycle not change
        ssrcInfo.biggestSeq = itemSeq;
        itemCycle = ssrcInfo.biggestCycle;
      } else {
        // case 2 recv old, but is last cycle
        if (ssrcInfo.biggestCycle <= 0) {
          // todo more logic, should return?
          tylog("recv shit packet, ignore it=%s.", rtpHeader.ToString().data());
          assert(!"should not reach here unless hacker attacks us, now we assert it");
        } else {
          itemCycle = ssrcInfo.biggestCycle - 1;  // notice
          tylog("recv old packet=%s. also old cycle",
                rtpHeader.ToString().data());
        }
      }
    } else {
      // itemSeq < ssrcInfo.biggestSeq
      // e.g. ssrcInfo.biggestSeq is 65535, itemSeq is 0
      if (AheadOf(itemSeq, ssrcInfo.biggestSeq)) {
        // case 3, recv newer, but rollback
        ssrcInfo.biggestSeq = itemSeq;
        ++ssrcInfo.biggestCycle;  // key
        itemCycle = ssrcInfo.biggestCycle;
        tylog("recv newer packet=%s. seq rollback",
              rtpHeader.ToString().data());
      } else {
        // case 4 recv old, same cycle
        tylog("recv old packet=%s.", rtpHeader.ToString().data());
        // ssrcInfo.biggestSeq, ssrcInfo.biggestCycle not change
        itemCycle = ssrcInfo.biggestCycle;
      }
    }

    // must save cycle for each packet, because may be different
    RtpBizPacket rtpBizPacket(
        std::move(const_cast<std::vector<char> &>(vBufReceive)), itemCycle);
    assert(vBufReceive.empty());

    // hack
    ssrcInfo.rtpReceiver.PushToJitter(std::move(rtpBizPacket));

    assert(rtpBizPacket.rtpRawPacket.empty());

    std::vector<RtpBizPacket> orderedPackets =
        ssrcInfo.rtpReceiver.PopOrderedPackets();

    tylog("pop jitter's OrderedPackets size=%zu", orderedPackets.size());

    for (RtpBizPacket &packet : orderedPackets) {
      ret = DumpPacketH264(packet.rtpRawPacket, ssrcInfo.h264Unpacketizer);
      if (ret) {
        tylog("dump uplink h264 ret=%d", ret);
        return ret;
      }

      ssrcInfo.rtpSender.Enqueue(std::move(packet));
      assert(packet.rtpRawPacket.empty());
    }

    // downlink
    std::vector<RtpBizPacket> sendPackets = ssrcInfo.rtpSender.Dequeue();

    tylog("send to peer packets size=%zu", sendPackets.size());
    for (RtpBizPacket &packet : sendPackets) {
      ret = SendToPeer_(packet.rtpRawPacket);
      if (ret) {
        tylog("send to peer ret=%d", ret);
        return ret;
      }
    }
  } else {
    tylog("receive unknown type of data=%s, return", mediaType.data());

    // should not use assert
    assert(!"receive unknown media type, should already filter and return");

    return -2;
  }

  return 0;
}

std::string RtpHandler::ToString() const {
  return "rtpDummyData";
  // return tylib::format_string( "{ssrcMap=%s}",
  // tylib::AnyToString(ssrc2unpacker_).data());
}
