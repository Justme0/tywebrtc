#include "rtp/rtp_handler.h"

#include <arpa/inet.h>

#include <cassert>
#include <cstring>
#include <string>

#include "global_tmp/global_tmp.h"
#include "log/log.h"
#include "pc/peer_connection.h"
#include "rtp/pack_unpack/rtp_to_h264.h"
#include "rtp/rtcp/rtcp_parser.h"
#include "rtp/rtp_parser.h"
#include "timer/timer.h"
#include "tylib/ip/ip.h"
#include "tylib/string/any_to_string.h"

// string enum, for print convenience
const std::string kMediaTypeRtcp = "rtcp";
const std::string kMediaTypeVideo = "video";
const std::string kMediaTypeAudio = "audio";

const AVRational kAudioTimebase{1, 48000};
const AVRational kVideoTimebase{1, 90000};

RtpHandler::RtpHandler(PeerConnection &pc)
    : belongingPeerConnection_(pc), videoTranscoder_(*this) {
  // opt: may have no audio
  SrsAudioCodecId from = SrsAudioCodecIdOpus;  // TODO: From SDP?
  SrsAudioCodecId to = SrsAudioCodecIdAAC;     // The output audio codec.
  int channels = 2;                            // The output audio channels.
  int sample_rate = 48000;  // The output audio sample rate in HZ.
  int bitrate = 64000;      // The output audio bitrate in bps.
  int ret =
      audioTranscoder_.initialize(from, to, channels, sample_rate, bitrate);
  if (ret) {
    tylog("init audio transcoder ret=%d", ret);

    assert(!"init audio transcoder fail, should not use assert :)");
  }

  ret = audioTranscoderDownlink_.initialize(
      SrsAudioCodecIdAAC, SrsAudioCodecIdOpus, 2, 48000, 64000);
  if (ret) {
    tylog("init downlink audio transcoder ret=%d", ret);

    assert(!"init downlink audio transcoder fail, should not use assert :)");
  }

  // init uplink file, should be class or function
  // check file cmd:
  // mediainfo filename.webm
  // ffmpeg -i filename.webm -f null -
  const std::string &webmFilename = tylib::format_string(
      "uplink.%s_%d.webm", belongingPeerConnection_.clientIP_.data(),
      belongingPeerConnection_.clientPort_);
  avformat_alloc_output_context2(&uplinkFileCtx_, nullptr, "webm",
                                 webmFilename.data());

  if (nullptr == uplinkFileCtx_) {
    tylog("init uplinkFile fail, should not use assert :)");
    assert(!"init uplinkFile fail, should not use assert :)");
  }

  // Add video stream
  AVStream *video_stream = avformat_new_stream(uplinkFileCtx_, nullptr);
  if (!video_stream) {
    tylog("init uplinkFile fail, should not use assert :)");
    assert(!"init uplinkFile fail, should not use assert :)");
  }
  video_stream->codecpar->codec_id = AV_CODEC_ID_VP8;
  video_stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
  // OPT: use const or config
  video_stream->codecpar->width = 450;
  video_stream->codecpar->height = 450;
  videoStreamIndex_ = video_stream->index;
  tylog("video stream index=%d.", videoStreamIndex_);

  // Add audio stream
  AVStream *audio_stream = avformat_new_stream(uplinkFileCtx_, nullptr);
  if (!audio_stream) {
    tylog("init uplinkFile fail, should not use assert :)");
    assert(!"init uplinkFile fail, should not use assert :)");
  }
  audio_stream->codecpar->codec_id = AV_CODEC_ID_OPUS;
  audio_stream->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
  audio_stream->codecpar->sample_rate = 48000;
  // OPT: use const or config
  audio_stream->codecpar->channel_layout = AV_CH_LAYOUT_MONO;
  audio_stream->codecpar->channels =
      av_get_channel_layout_nb_channels(audio_stream->codecpar->channel_layout);
  audioStreamIndex_ = audio_stream->index;
  tylog("audio stream index=%d.", audioStreamIndex_);

  // Open the output file
  assert(!(uplinkFileCtx_->oformat->flags & AVFMT_NOFILE));
  ret = avio_open(&uplinkFileCtx_->pb, uplinkFileCtx_->url, AVIO_FLAG_WRITE);
  if (ret < 0) {
    tylog("cannot open avio, url=%s, ret=%d[%s]", uplinkFileCtx_->url, ret,
          av_err2string(ret));

    // use goto error?
    avformat_free_context(uplinkFileCtx_);
    uplinkFileCtx_ = nullptr;

    assert(!"init uplinkFile fail, should not use assert :)");
  }

  // Write the file header
  ret = avformat_write_header(uplinkFileCtx_, nullptr);
  if (ret < 0) {
    tylog("write header ret=%d[%s]", ret, av_err2string(ret));
    assert(!"init uplinkFile fail, should not use assert :)");
  }
}

RtpHandler::~RtpHandler() {
  if (uplinkFileCtx_ != nullptr) {
    av_write_trailer(uplinkFileCtx_);
    avio_close(uplinkFileCtx_->pb);
    avformat_free_context(uplinkFileCtx_);
  }
}

static int WriteFile(const std::vector<char> &data) {
  static FILE *pfOutfpAAC = nullptr;
  if (nullptr == pfOutfpAAC) {
    pfOutfpAAC = fopen("audio.aac", "wb");
    if (nullptr == pfOutfpAAC) {
      tylog("open aac file fail, errno=%d[%s]", errno, strerror(errno));
      // should not use assert
      assert(!"shit open aac file");

      return -1;
    }
  }

  size_t n = fwrite(data.data(), data.size(), 1, pfOutfpAAC);
  if (n < 1) {
    tylog("fwrite fail, return value=%zu.", n);
    return -2;
  }

  return 0;
}

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

// write to file, should add more check
int RtpHandler::WriteWebmFile(const std::string &frame, uint32_t rtpTs,
                              const std::string &mediaType, bool bKeyFrame) {
  int ret = 0;

  int rtpPTS = 0;
  int streamIndex = 0;
  AVRational timebase;
  if (kMediaTypeAudio == mediaType) {
    rtpPTS = rtpTs - firstRtpAudioTs_;
    streamIndex = audioStreamIndex_;
    timebase = kAudioTimebase;
  } else {
    rtpPTS = rtpTs - firstRtpVideoTs_;
    streamIndex = videoStreamIndex_;
    timebase = kVideoTimebase;
  }

  AVPacket avPacket;  // not alloc,may memory leak?
  av_init_packet(&avPacket);
  avPacket.data = reinterpret_cast<uint8_t *>(const_cast<char *>(frame.data()));
  avPacket.size = frame.size();
  if (bKeyFrame) {
    avPacket.flags |= AV_PKT_FLAG_KEY;
  }
  avPacket.stream_index = streamIndex;
  // avPacket.time_base = AVRational{1, 1000};

  const int64_t pts = av_rescale_q(
      rtpPTS, timebase, uplinkFileCtx_->streams[streamIndex]->time_base);
  avPacket.pts = pts;
  avPacket.dts = pts;

  tylog("stmIdx=%d, rtpPTS=%d, timebase %d / %d, now_ms=%s, pts=%ld.",
        streamIndex, rtpPTS,
        uplinkFileCtx_->streams[streamIndex]->time_base.num,
        uplinkFileCtx_->streams[streamIndex]->time_base.den,
        tylib::MilliSecondToLocalTimeString(g_now_ms).data(), pts);

  ret = av_interleaved_write_frame(uplinkFileCtx_, &avPacket);
  if (ret < 0) {
    tylog("vp8 interleaved_write_frame ret=%d[%s].", ret, av_err2string(ret));

    return ret;
  }

  return 0;
}

// dump 264 or rtmp push
int RtpHandler::DumpPacket(const std::vector<char> &packet,
                           H264Unpacketizer &unpacker) {
  int ret = 0;
  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(packet.data());

  std::string mediaType = rtpHeader.GetMediaType();

  if (mediaType == kMediaTypeVideo) {
    if (0 == firstRtpVideoTs_) {
      firstRtpVideoTs_ = rtpHeader.getTimestamp();
    }

    std::vector<std::string> h264Frames;  // should with dts
    const bool isvp8 = true;              // tmp
    if (isvp8) {
      ret = this->videoTranscoder_.VideoUnPackVp8RtpStm(
          packet.data(), packet.size(), &h264Frames);
      if (ret) {
        tylog("vp8 decode ret=%d.", ret);

        return ret;
      }
    } else {
      std::vector<MediaData> media;
      ret = unpacker.Unpacketize(packet, &media);
      if (ret) {
        tylog("unpacketize rtp ret=%d", ret);
        return ret;
      }
      for (MediaData &m : media) {
        h264Frames.emplace_back(std::move(m.data_));
        assert(m.data_.empty());
      }
    }

    tylog("video h264 frames.size=%zu.", h264Frames.size());
    assert(h264Frames.size() <= 1);  // tmp
    for (const std::string &frame : h264Frames) {
      ret = unpacker.DumpRawStream(frame, rtpHeader.getSSRC());
      if (ret) {
        tylog("dump raw stream ret=%d", ret);

        return ret;
      }

      ret = this->belongingPeerConnection_.pushHandler_.SendVideoFrame(
          std::vector<char>(frame.begin(), frame.end()),
          (rtpHeader.getTimestamp() - firstRtpVideoTs_) / 90);
      if (ret) {
        tylog("push send video ret=%d", ret);

        return ret;
      }
    }
  } else {
    if (0 == firstRtpAudioTs_) {
      firstRtpAudioTs_ = rtpHeader.getTimestamp();
    }

    assert(mediaType == kMediaTypeAudio);

    const char *payloadBegin = packet.data() + rtpHeader.getHeaderLength();
    const char *payloadEnd =
        packet.data() + packet.size() - getRtpPaddingLength(packet);

    int64_t nowMs = g_now_ms;

    tylog("before audio transcode, time(ms)=%ld, opus size=%ld.", nowMs,
          payloadEnd - payloadBegin);
    assert(payloadBegin < payloadEnd);  // may have probe packet(size=0) ?

    ret = WriteWebmFile({payloadBegin, payloadEnd}, rtpHeader.getTimestamp(),
                        kMediaTypeAudio, false);
    if (ret) {
      tylog("write webm audio file ret=%d.", ret);

      return ret;
    }

    SrsAudioFrame f;
    // OPT: avoid copy
    f.s.assign(payloadBegin, payloadEnd);
    f.ts_ms = nowMs;

    std::vector<SrsAudioFrame> outFrames;
    ret = this->audioTranscoder_.transcode(f, outFrames);
    if (ret) {
      tylog("audio transcode ret=%d", ret);

      return ret;
    }

    tylog("transcode opus->aac return succ, outFrames.size=%zu.",
          outFrames.size());
    assert(outFrames.size() <= 1);  // tmp

    for (const SrsAudioFrame &outFrame : outFrames) {
      // to use string

      // https://stackoverflow.com/questions/65013622/ffmpeg-encoding-aac-audio-encoded-file-can-not-be-played#comment115182982_65150073

      // https://wiki.multimedia.cx/index.php/MPEG-4_Audio#Channel_Configurations
      int aac_profile = 2;            // AAC LC
      int frequencey_index = 3;       // 48000 Hz
      int channel_configuration = 2;  // stereo (left, right)

      unsigned char adts_header[7];
      const int kADTSHeaderLen = sizeof adts_header;

      // KEY: Frame length, length of the ADTS frame including headers and CRC
      // check.
      // we have no CRC check.
      int frame_length = outFrame.s.size() + kADTSHeaderLen;

      // Take look here: https://wiki.multimedia.cx/index.php/ADTS
      // fill in ADTS data
      adts_header[0] = (unsigned char)0xFF;
      adts_header[1] = (unsigned char)0xF9;
      adts_header[2] =
          (unsigned char)(((aac_profile - 1) << 6) + (frequencey_index << 2) +
                          (channel_configuration >> 2));
      adts_header[3] = (unsigned char)(((channel_configuration & 3) << 6) +
                                       (frame_length >> 11));
      adts_header[4] = (unsigned char)((frame_length & 0x7FF) >> 3);
      adts_header[5] = (unsigned char)(((frame_length & 7) << 5) + 0x1F);
      adts_header[6] = (unsigned char)0xFC;

      std::vector<char> adts_frame(adts_header, adts_header + kADTSHeaderLen);
      adts_frame.insert(adts_frame.end(), outFrame.s.begin(), outFrame.s.end());

      WriteFile(adts_frame);

      ret = this->belongingPeerConnection_.pushHandler_.SendAudioFrame(
          adts_frame, (rtpHeader.getTimestamp() - firstRtpAudioTs_) / 48);
      if (ret) {
        tylog("rtmp send audio ret=%d", ret);
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

// to rename, now called in only one position
int RtpHandler::SendToPeer_(RtpBizPacket &rtpBizPacket) {
  auto peerPC = belongingPeerConnection_.FindPeerPC();
  if (nullptr == peerPC) {
    tylog("found no other peer");

    return 0;
  }

  tylog("found other peer=%s.", peerPC->ToString().data());

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
    downlinkRtpHeader.setPayloadType(peerPC->sdpHandler_.vp8PayloadType);
  } else {
    tylog("invalid downlink send media type=%s.", mediaType.data());
    assert(!"invalid media type");
  }

  DumpSendPacket(rtpBizPacket.rtpRawPacket);

  ret = peerPC->srtpHandler_.ProtectRtp(&rtpBizPacket.rtpRawPacket);
  if (ret) {
    tylog("downlink protect rtp ret=%d", ret);

    return ret;
  }

  // OPT: cancle string copy
  std::vector<char> saveRtp(rtpBizPacket.rtpRawPacket);

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

  ret = peerPC->SendToClient(saveRtp);
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

  if (belongingPeerConnection_.stateMachine_ < EnumStateMachine::DTLS_DONE) {
    tylog("warning: recv rtp, but now state=%s, should be DTLS_DONE!!!",
          StateMachineToString(belongingPeerConnection_.stateMachine_).data());
    return -1;
  } else if (belongingPeerConnection_.stateMachine_ ==
             EnumStateMachine::DTLS_DONE) {
    // notify others I entered
    belongingPeerConnection_.stateMachine_ = EnumStateMachine::GOT_RTP;
    tylog("stateMachine=%s, handShakeCompleted",
          StateMachineToString(belongingPeerConnection_.stateMachine_).data());

    auto peerPC = belongingPeerConnection_.FindPeerPC();
    if (nullptr != peerPC) {
      peerPC->dataChannelHandler_.SendSctpDataForLable("I'm coming.");
    }

    // init push handler
    const char *url = std::getenv("TY_PUSH_URL");
    if (nullptr != url) {
      tylog("push url=%s", url);

      RtmpHandler &rtmpPusher =
          *new RtmpHandler(this->belongingPeerConnection_);  // FIXME

      ret = this->belongingPeerConnection_.pushHandler_.InitPushHandler(
          std::bind(&RtmpHandler::InitProtocolHandler, &rtmpPusher, url),
          std::bind(&RtmpHandler::InitSucc, &rtmpPusher),
          std::bind(&RtmpHandler::SendAudioFrame, &rtmpPusher,
                    std::placeholders::_1, std::placeholders::_2),
          std::bind(&RtmpHandler::SendVideoFrame, &rtmpPusher,
                    std::placeholders::_1, std::placeholders::_2));
      if (ret) {
        tylog("Handler.handshakeTo ret=%d.", ret);

        // return ret;
      }
    } else {
      tylog("push url env var not exist");
    }

    // if pull fail, retry?
    // but current branch is run only once
    url = std::getenv("TY_PULL_URL");
    if (nullptr != url) {
      tylog("pull url=%s", url);

      RtmpPuller &rtmpPuller =
          *new RtmpPuller(this->belongingPeerConnection_);  // FIXME

      ret = this->belongingPeerConnection_.pullHandler_.InitPullHandler(
          &rtmpPuller.rtmp_.m_sb.sb_socket,
          std::bind(&RtmpPuller::InitProtocolHandler, &rtmpPuller, url),
          std::bind(&RtmpPuller::HandlePacket, &rtmpPuller),
          std::bind(&RtmpPuller::CloseStream, &rtmpPuller));
      if (ret) {
        tylog("Handler.handshakeTo ret=%d.", ret);

        // return ret;
      }
    } else {
      tylog("pull url env var not exist");
    }

    // 收到RTP后定时请求I帧
    TimerManager::Instance()->AddTimer(
        &this->belongingPeerConnection_.pliTimer_);
  }

  assert(belongingPeerConnection_.stateMachine_ == EnumStateMachine::GOT_RTP);

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

    // OPT: SSRC should be assigned from SDP, now should to check
    if (mediaType == kMediaTypeAudio) {
      this->upAudioSSRC = rtpHeader.getSSRC();
    } else if (mediaType == kMediaTypeVideo) {
      this->upVideoSSRC = rtpHeader.getSSRC();
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
          tylog("recv shit packet(usually sender not browser, e.g. server)=%s.",
                rtpHeader.ToString().data());
          // web browser first seq is always AheadOf 0
          // assert(!"should not reach here unless hacker attacks us, now we
          // assert it");
          itemCycle = 0;
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

    if (rtpBizPacket.GetPowerSeq() < ssrcInfo.rtpReceiver.lastPoppedPowerSeq_) {
      tylog("current=%s < lastPop=%s, ignore current pkt.",
            PowerSeqToString(rtpBizPacket.GetPowerSeq()).data(),
            PowerSeqToString(ssrcInfo.rtpReceiver.lastPoppedPowerSeq_).data());

      return 0;
    }

    ssrcInfo.rtpReceiver.PushToJitter(std::move(rtpBizPacket));

    assert(rtpBizPacket.rtpRawPacket.empty());

    std::vector<RtpBizPacket> orderedPackets =
        ssrcInfo.rtpReceiver.PopOrderedPackets();

    tylog("pop jitter's OrderedPackets size=%zu", orderedPackets.size());

    for (RtpBizPacket &packet : orderedPackets) {
      // push to other server
      if (this->belongingPeerConnection_.pushHandler_.InitSucc()) {
        // OPT: first frame should be I frame; if lost packet should drop the
        // gop
        ret = DumpPacket(packet.rtpRawPacket, ssrcInfo.h264Unpacketizer);
        if (ret) {
          tylog("dump uplink packet ret=%d, not return err :)", ret);
          // return ret;
        }
      }

      // send to peer
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
  return tylib::format_string(
      "upAudioSSRC=%u, upVideoSSRC=%u, ssrcMapSize=%zu.", upAudioSSRC,
      upVideoSSRC, ssrcInfoMap_.size());
}
