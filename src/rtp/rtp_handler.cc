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
#include "timer/timer.h"

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
  // temp return
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

SSRCInfo::SSRCInfo(RtpHandler &belongingRtpHandler)
    : rtpReceiver(*this),
      rtpSender(*this),
      belongingRtpHandler(belongingRtpHandler) {}

std::string SSRCInfo::ToString() const {
  return tylib::format_string("{biggestSeq=%u, biggestCycle=%ld}", biggestSeq,
                              biggestCycle);
}

// tmp
inline std::shared_ptr<PeerConnection> getPeerPC(const std::string &selfIP,
                                                 int selfPort) {
  for (const auto &p : Singleton::Instance().client2PC_) {
    if (selfIP == p.first.ip && selfPort == p.first.port) {
      continue;
    }

    if (p.second->stateMachine_ < EnumStateMachine::GOT_RTP) {
      continue;
    }

    return p.second;
  }

  return nullptr;
}

// to rename, now called in only one position
int RtpHandler::SendToPeer_(RtpBizPacket &rtpBizPacket) {
  int ret = 0;

  RtpHeader &downlinkRtpHeader =
      *reinterpret_cast<RtpHeader *>(rtpBizPacket.rtpRawPacket.data());

  std::string mediaType = downlinkRtpHeader.GetMediaType();
  tylog("downlink send media type=%s.", mediaType.data());

  if (mediaType == kMediaTypeAudio) {
    // audio not use pacing
    downlinkRtpHeader.setSSRC(kDownlinkAudioSsrc);
    downlinkRtpHeader.setPayloadType(kDownlinkAudioPayloadType);
  } else if (mediaType == kMediaTypeVideo) {
    downlinkRtpHeader.setSSRC(kDownlinkVideoSsrc);
    downlinkRtpHeader.setPayloadType(kDownlinkVideoVp8PayloadType);
  } else {
    tylog("invalid downlink send media type=%s.", mediaType.data());
    assert(!"invalid media type");
  }

  DumpSendPacket(rtpBizPacket.rtpRawPacket);

  auto peerPC = getPeerPC(belongingPeerConnection_.clientIP_,
                          belongingPeerConnection_.clientPort_);
  if (nullptr == peerPC) {
    tylog("found no other peer");

    return 0;
  }

  tylog("found other peer=%s.", peerPC->ToString().data());

  ret = peerPC->srtpHandler_.ProtectRtp(
      const_cast<std::vector<char> *>(&rtpBizPacket.rtpRawPacket));
  if (ret) {
    tylog("downlink protect rtp ret=%d", ret);

    return ret;
  }

  // OPT: cancle string copy
  std::vector<char> tmpbuf(rtpBizPacket.rtpRawPacket);

  tylog("downlink rtp=%s", downlinkRtpHeader.ToString().data());

  auto it = peerPC->rtpHandler_.ssrcInfoMap_.find(downlinkRtpHeader.getSSRC());
  if (peerPC->rtpHandler_.ssrcInfoMap_.end() == it) {
    auto p = peerPC->rtpHandler_.ssrcInfoMap_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(downlinkRtpHeader.getSSRC()),
        std::forward_as_tuple(peerPC->rtpHandler_));
    assert(p.second);
    it = p.first;
    assert(&it->second == &it->second.rtpReceiver.belongingSSRCInfo_);
  }
  SSRCInfo &ssrcInfo = it->second;
  ssrcInfo.rtpSender.Enqueue(std::move(rtpBizPacket));

  ret = peerPC->SendToClient(tmpbuf);
  if (ret) {
    tylog("send to peer ret=%d", ret);

    return ret;
  }

  return 0;
}

int RtpHandler::HandleRtpPacket(const std::vector<char> &vBufReceive) {
  g_recvPacketNum->Add({{"dummy", "recv"}}).Increment();

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

  if (belongingPeerConnection_.stateMachine_ < EnumStateMachine::GOT_RTP) {
    belongingPeerConnection_.stateMachine_ = EnumStateMachine::GOT_RTP;

    // 收到RTP后定时请求I帧
    TimerManager::Instance()->AddTimer(
        &this->belongingPeerConnection_.pcTimer_);
  }

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

    DumpRecvPacket(vBufReceive);

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
      int r = rand() % 100;
      if (r < kUplossRateMul100) {
        tylog("up rand=%d lostrate=%d%%, drop! rtp=%s.", r, kUplossRateMul100,
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
      tylog("warning: unprotect RTP (not RTCP) fail ret=%d", ret);

      return ret;
    }
    DumpRecvPacket(vBufReceive);
    tylog("after unprotect, paddinglen=%d", getRtpPaddingLength(vBufReceive));

    const RtpHeader &rtpHeader =
        *reinterpret_cast<const RtpHeader *>(vBufReceive.data());
    tylog("recv rtp=%s.", rtpHeader.ToString().data());

    if (mediaType == kMediaTypeAudio) {
      g_UplinkAudioSsrc = rtpHeader.getSSRC();
    } else if (mediaType == kMediaTypeVideo) {
      g_UplinkVideoSsrc = rtpHeader.getSSRC();
    }

    // Constructing a value SSRCInfo is expensive, so we should not insert
    // directly. Instead find firstly.
    // OPT: use lower_bound and emplace with hint (ref to
    // Singleton::GetPeerConnection). Because hash emplace is amortized O(1), so
    // we do not use hint currently. Using find is more readable.
    auto it = ssrcInfoMap_.find(rtpHeader.getSSRC());
    if (ssrcInfoMap_.end() == it) {
      // second query key, but O(1).
      // NOTE that SSRCInfo has no default constructor.
      // What if rehash make original SSRCInfo address changed?
      // belongingRtpHandler ref to original address.
      //
      // ref:
      // maybe not perfect:
      // https://stackoverflow.com/questions/1935139/using-stdmapk-v-where-v-has-no-usable-default-constructor
      // good: https://juejin.cn/post/7029372430397210632
      auto p = ssrcInfoMap_.emplace(std::piecewise_construct,
                                    std::forward_as_tuple(rtpHeader.getSSRC()),
                                    std::forward_as_tuple(*this));
      assert(p.second);
      it = p.first;
      assert(&it->second == &it->second.rtpReceiver.belongingSSRCInfo_);
    }
    SSRCInfo &ssrcInfo = it->second;
    assert(&ssrcInfo == &ssrcInfo.rtpReceiver.belongingSSRCInfo_);

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

      ret = SendToPeer_(packet);
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
  return tylib::format_string("ssrcMapSize=%zu.", ssrcInfoMap_.size());
}
