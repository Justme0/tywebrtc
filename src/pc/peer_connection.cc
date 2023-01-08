#include "pc/peer_connection.h"

#include "log/log.h"
#include "tylib/ip/ip.h"
#include "tylib/time/time_util.h"
#include "tylib/time/timer.h"

PeerConnection::PeerConnection()
    : stateMachine_(EnumStateMachine::GOT_CANDIDATE),  // sdp has candiate
      sdpHandler_(*this),
      iceHandler_(*this),
      dtlsHandler_(*this, false),  // taylor 写死 dtls client
      rtpHandler_(*this),
      rtcpHandler_(*this),
      srtpHandler_(*this),
      initTimeMs_(g_now_ms) {}

// vBufSend is crypto data
int PeerConnection::SendToClient(const std::vector<char> &vBufSend) const {
  sockaddr_in addr = tylib::ConstructSockAddr(clientIP_, clientPort_);
  ssize_t sendtoLen =
      sendto(g_sock_fd, vBufSend.data(), vBufSend.size(), 0,
             reinterpret_cast<sockaddr *>(&addr), sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto ret=-1 errorno=%d[%s]", errno, strerror(errno));
    return -1;
  }
  tylog("sendto succ buf size=%ld, ip=%s, port=%d.", sendtoLen,
        clientIP_.data(), clientPort_);

  return 0;
}

// vBufSend is crypto data
int PeerConnection::SendToPeer(const std::vector<char> &vBufSend) const {
  assert(!"shit");
  /*

    sockaddr_in addr = tylib::ConstructSockAddr(peerIP, peerPort);
    ssize_t sendtoLen =
        sendto(g_sock_fd, vBufSend.data(), vBufSend.size(), 0,
               reinterpret_cast<sockaddr *>(&addr), sizeof(struct sockaddr_in));
    if (-1 == sendtoLen) {
      tylog("sendto ret=-1 errorno=%d[%s]", errno, strerror(errno));

      return errno;
    }

    tylog("sendto succ buf size=%ld, ip=%s, port=%d.", sendtoLen, peerIP.data(),
          peerPort);
          */

  return 0;
}

int PeerConnection::HandlePacket(const std::vector<char> &vBufReceive) {
  this->lastActiveTimeMs_ = g_now_ms;

  int ret = 0;

  uint8_t cSubCmd = vBufReceive.front();
  PacketType packType = getPacketType(cSubCmd);
  tylog("subcmd=%hhu, packType=%s", cSubCmd,
        PacketTypeToString(packType).data());
  tylog("stateMachine=%s", StateMachineToString(stateMachine_).data());

  // packType number is little, so we don't use map-callback style, just
  // switch-case
  switch (packType) {
    case PacketType::STUN: {
      ret = iceHandler_.HandleIcePacket(vBufReceive);
      if (ret) {
        tylog("handle ice packet fail, ret=%d", ret);
        return ret;
      }
      break;
    }

    case PacketType::DTLS: {
      ret = dtlsHandler_.HandleDtlsPacket(vBufReceive);
      if (ret) {
        tylog("handle dtls packet fail, ret=%d", ret);
        return ret;
      }
      break;
    }

    case PacketType::RTP: {
      ret = rtpHandler_.HandleRtpPacket(vBufReceive);
      if (ret) {
        tylog("handle rtp packet fail, ret=%d", ret);
        return ret;
      }
      break;
    }

    default:
      tylog("unknown packet type %s", PacketTypeToString(packType).data());
      break;
  }

  return 0;
}

// recv another server's packet
int HandleDownlinkPacket(const std::vector<char> &vBufReceive) {
  int ret = 0;
  RtpHeader &downlinkRtpHeader =
      *reinterpret_cast<RtpHeader *>(vBufSend.data());

  uint32_t remoteSSRC = downlinkRtpHeader.getSSRC();
  std::string mediaType = downlinkRtpHeader.GetMediaType();
  tylog("downlink send media type=%s.", mediaType.data());

  // payload type not changed
  if (mediaType == kMediaTypeAudio) {
    // taylor audio not use pacing
    downlinkRtpHeader.setSSRC(kDownlinkAudioSsrc);
    // downlinkRtpHeader.setPayloadType(kDownlinkAudioPayloadType);
  } else if (mediaType == kMediaTypeVideo) {
    downlinkRtpHeader.setSSRC(kDownlinkVideoSsrc);
    // downlinkRtpHeader.setPayloadType(kDownlinkVideoVp8PayloadType);
  } else {
    tylog("invalid downlink send media type=%s.", mediaType.data());
    assert(!"invalid media type");
  }

  // to extract common
  // Constructing a value SSRCInfo is expensive, so we should not insert
  // directly. Instead find firstly.
  // OPT: use lower_bound and emplace with hint (ref to
  // Singleton::GetPeerConnection). Because hash emplace is amortized O(1), so
  // we do not use hint currently. Using find is more readable.
  auto it = rtpHandler_.ssrcInfoMap_.find(downlinkRtpHeader.getSSRC());
  if (rtpHandler_.ssrcInfoMap_.end() == it) {
    // second query key, but O(1).
    // NOTE that SSRCInfo has no default constructor.
    // What if rehash make original SSRCInfo address changed?
    // belongingRtpHandler ref to original address.
    //
    // ref:
    // maybe not perfect:
    // https://stackoverflow.com/questions/1935139/using-stdmapk-v-where-v-has-no-usable-default-constructor
    // good: https://juejin.cn/post/7029372430397210632
    auto p = rtpHandler_.ssrcInfoMap_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(downlinkRtpHeader.getSSRC()),
        std::forward_as_tuple(rtpHandler_));

 // should be in ssrcInfo constructor
    ssrcInfo.rtpSender.remoteSSRC = remoteSSRC;

    assert(p.second);
    it = p.first;
    assert(&it->second == &it->second.rtpReceiver.belongingSSRCInfo_);
  }
  SSRCInfo &ssrcInfo = it->second;
  assert(&ssrcInfo == &ssrcInfo.rtpReceiver.belongingSSRCInfo_);

  tylog("rtpHandler=%s.", rtpHandler_.ToString().data());

  downlinkRtpHeader.setSeqNumber(ssrcInfo.downlinkSeq++);

  DumpSendPacket(vBufSend);

  ret = srtpHandler_.ProtectRtp(const_cast<std::vector<char> *>(&vBufSend));
  if (ret) {
    tylog("downlink protect rtp ret=%d", ret);
    return ret;
  }

  if (ssrcInfo.rtpSender.remoteSSRC != remoteSSRC) {

  }
  ssrcInfo.rtpSender.Enqueue(std::move(packet));
  std::vector<RtpBizPacket> sendPackets = ssrcInfo.rtpSender.Dequeue();

  // test video loss, should after srtp
  if (mediaType == kMediaTypeVideo) {
    int r = rand() % 100;
    if (r < kDownlossRateMul100) {
      tylog("down rand=%d lostrate=%d%%, drop! after protect rtp=%s.", r,
            kDownlossRateMul100,
            reinterpret_cast<const RtpHeader *>(vBufReceive.data())
                ->ToString()
                .data());

      return 0;
    }
  }

  ret = SendToClient(vBufSend);
  if (ret) {
    tylog("send to peer ret=%d", ret);

    return ret;
  }

  return 0;
}

std::string PeerConnection::ToString() const {
  return tylib::format_string(
      "{state=%s, client=%s:%d, initTime=%s, lastLiveTime=%s}",
      StateMachineToString(stateMachine_).data(), clientIP_.data(), clientPort_,
      tylib::MilliSecondToLocalTimeString(initTimeMs_).data(),
      tylib::MilliSecondToLocalTimeString(lastActiveTimeMs_).data());
}
