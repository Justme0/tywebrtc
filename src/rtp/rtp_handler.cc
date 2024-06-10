// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtp_handler.h"

#include <arpa/inet.h>

#include <cassert>
#include <cstring>
#include <string>

#include "rsfec/rsfec.h"
#include "tylib/ip/ip.h"
#include "tylib/string/any_to_string.h"

#include "src/global_tmp/global_tmp.h"
#include "src/log/log.h"
#include "src/pc/peer_connection.h"
#include "src/rtp/pack_unpack/rtp_to_h264.h"
#include "src/rtp/rtcp/rtcp_parser.h"
#include "src/rtp/rtp_parser.h"
#include "src/timer/timer.h"

namespace tywebrtc {

// string enum, for print convenience
const std::string kMediaTypeRtcp = "rtcp";
const std::string kMediaTypeVideo = "video";
const std::string kMediaTypeAudio = "audio";

const AVRational kAudioTimebase{1, kAudioPayloadTypeFrequency};
const AVRational kVideoTimebase{1, kVideoPayloadTypeFrequency};

RtpHandler::RtpHandler(PeerConnection &pc)
    : belongingPC_(pc), videoTranscoder_(*this) {
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
  const std::string &webmFilename =
      tylib::format_string("uplink.%s_%d.webm", belongingPC_.clientIP().data(),
                           belongingPC_.clientPort());
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
  audio_stream->codecpar->ch_layout = AV_CHANNEL_LAYOUT_MONO;
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

  std::string mediaType = rtpHeader.ComputeMediaType();

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

      ret = this->belongingPC_.pushHandler_.SendVideoFrame(
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

      ret = this->belongingPC_.pushHandler_.SendAudioFrame(
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

SSRCInfo::SSRCInfo(RtpHandler &belongingRtpHandler, uint32_t ssrc,
                   bool is_audio)
    : rtpReceiver(*this),
      rtpSender(*this),
      belongingRtpHandler(belongingRtpHandler),
      ssrc_key_(ssrc),
      is_audio_(is_audio) {}

std::vector<std::vector<char>> SSRCInfo::EncodeFec(
    const std::vector<RtpBizPacket> &rtpBizPackets) {
  const int originalNum = rtpBizPackets.size();
  if (originalNum == 0) {
    tylog("warn: originalNum=0");

    return {};
  }

  if (originalNum > kMatrixMaxSize) {
    // taylor to support
    tylog("warn: originalNum=%d>%d, not encode fec.", originalNum,
          kMatrixMaxSize);

    return {};
  }

  const RtpHeader &first = *reinterpret_cast<const RtpHeader *>(
      rtpBizPackets.front().rtpRawPacket.data());
  const uint16_t firstSeq = first.getSeqNumber();
  const RtpHeader &last = *reinterpret_cast<const RtpHeader *>(
      rtpBizPackets.back().rtpRawPacket.data());
  const uint16_t lastSeq = last.getSeqNumber();
  const uint32_t originalSSRC = first.getSSRC();

  const int fecNum = ceil(kFecRate * originalNum);
  assert(fecNum >= 1);
  assert(fecNum <= kMatrixMaxSize);  // FIXME: allow more FEC

  // OPT: init encode matrix once
  rsfec::CRSFec rsfec;
  rsfec.SetVandermondeMatrix();
  int ret = rsfec.SetNM(originalNum, fecNum);
  if (ret) {
    tylog("setNM ret=%d, originalNum=%d, fecNum=%d.", ret, originalNum, fecNum);
    assert(0);  // should not use assert, for debug

    return {};
  }

  tylog("setNM originalNum=%d, fecNum=%d.", originalNum, fecNum);

  size_t maxPacketSize = 0;  // will also be fec payload length
  for (int i = 0; i < originalNum; ++i) {
    if (rtpBizPackets[i].rtpRawPacket.size() > maxPacketSize) {
      maxPacketSize = rtpBizPackets[i].rtpRawPacket.size();
    }
  }

  std::vector<const void *> inPtr(originalNum);
  std::vector<std::vector<char>> supplement(originalNum);
  std::vector<int> srcPktLengths(originalNum);
  for (int i = 0; i < originalNum; ++i) {
    srcPktLengths[i] = rtpBizPackets[i].rtpRawPacket.size();

    if (rtpBizPackets[i].rtpRawPacket.size() < maxPacketSize) {
      supplement[i] = rtpBizPackets[i].rtpRawPacket;
      supplement[i].resize(maxPacketSize);
      inPtr[i] = supplement[i].data();
    } else {
      assert(rtpBizPackets[i].rtpRawPacket.size() == maxPacketSize);
      inPtr[i] = rtpBizPackets[i].rtpRawPacket.data();
    }
  }

  std::vector<void *> outPtr(fecNum);
  std::vector<std::vector<uint8_t>> fecData(fecNum);
  for (int i = 0; i < fecNum; ++i) {
    fecData[i].resize(maxPacketSize);
    outPtr[i] = fecData[i].data();
  }

  ret = rsfec.EncodeFEC(maxPacketSize, inPtr, outPtr);
  if (ret) {
    tylog("calculateFEC ret=%d.", ret);
    assert(0);  // should not use assert, for debug
    return {};
  }

  std::vector<std::vector<char>> fecPackets;

  const int kFecHeadLen = 2 + 2 + 4 + 2 + 2 + (srcPktLengths.size() * 2);
  const int kAllLen = kRtpHeaderLenByte + kFecHeadLen + maxPacketSize;
  tylog("kAllLen=%d(%d+%d+%zu).", kAllLen, kRtpHeaderLenByte, kFecHeadLen,
        maxPacketSize);
  if (kAllLen > MAX_PKT_BUF_SIZE) {
    tylog("error: fec pkt too large, size=%d > %d.", kAllLen, MAX_PKT_BUF_SIZE);
    // should use monitor
    assert(!"fec pkt too large");
    return {};
  }

  for (int i = 0; i < fecNum; ++i) {
    std::vector<char> packet(kAllLen);

    RtpHeader &header = *reinterpret_cast<RtpHeader *>(packet.data());
    header.setVersion(2);                      // fix number
    header.setTimestamp(last.getTimestamp());  // same as last pkt
    header.setSSRC(this->ssrc_key_);
    header.setSeqNumber(
        h264Packetizer.GeneratePowerSequence());          // should use seq
    header.setPayloadType(kDownlinkVideoFecPayloadType);  // same as video
    header.setMarker(i == fecNum - 1 ? 1 : 0);
    header.setExtension(0);  // use user-defined extension

    assert(kRtpHeaderLenByte == header.getHeaderLength());

    // have no CSRC, not use extension, use user defined:
    // wide is 32B
    // fec header:
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // | FEC number            |     FEC index (0-based)   |
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // |        protected ssrc                             |
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // | protected first seq   | protected last seq        |
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // | protected data pkt len ...         | length is (last - first + 1) * 2B
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // |    FEC payload ...                        |
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    //
    // FEC number:  一组FEC一共几个包
    // FEC index:  FEC组里当前的index
    // protect ssrc:  保护的媒体的ssrc
    // first protect seq number 和 last protect seq number:
    // 这个FEC包是哪几个媒体包生成的

    char *fecHeader = packet.data() + kRtpHeaderLenByte;

    *reinterpret_cast<uint16_t *>(fecHeader) = htons(fecNum);
    fecHeader += 2;
    *reinterpret_cast<uint16_t *>(fecHeader) = htons(i);
    fecHeader += 2;

    *reinterpret_cast<uint32_t *>(fecHeader) = htonl(originalSSRC);
    fecHeader += 4;

    *reinterpret_cast<uint16_t *>(fecHeader) = htons(firstSeq);
    fecHeader += 2;
    *reinterpret_cast<uint16_t *>(fecHeader) = htons(lastSeq);
    fecHeader += 2;

    // OPT: how to cancel the field:
    for (size_t srcLen : srcPktLengths) {
      assert(srcLen <= std::numeric_limits<uint32_t>::max());
      *reinterpret_cast<uint16_t *>(fecHeader) = htons(srcLen);
      fecHeader += 2;
    }

    char *fecPayload = fecHeader;

    assert(maxPacketSize == fecData[i].size());
    memcpy(fecPayload, fecData[i].data(), fecData[i].size());

    fecPackets.emplace_back(packet);
  }

  return fecPackets;
}

std::string SSRCInfo::ToString() const {
  return tylib::format_string("{biggestSeq=%u, biggestCycle=%ld}", biggestSeq,
                              biggestCycle);
}

// to rename, now called in only one position
int RtpHandler::SendToPeer_(RtpBizPacket &rtpBizPacket) {
  auto peerPC = belongingPC_.FindPeerPC();
  if (nullptr == peerPC) {
    tylog("found no other peer");

    return 0;
  }

  tylog("found other peer=%s.", peerPC->ToString().data());

  int ret = 0;

  RtpHeader &downlinkRtpHeader =
      *reinterpret_cast<RtpHeader *>(rtpBizPacket.rtpRawPacket.data());
  std::string mediaType = downlinkRtpHeader.ComputeMediaType();
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
        std::forward_as_tuple(peerPC->rtpHandler_, downlinkRtpHeader.getSSRC(),
                              mediaType == kMediaTypeAudio));
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

  if (belongingPC_.stateMachine_ < EnumStateMachine::DTLS_DONE) {
    tylog("warning: recv rtp, but now state=%s, should be DTLS_DONE!!!",
          StateMachineToString(belongingPC_.stateMachine_).data());
    return -1;
  } else if (belongingPC_.stateMachine_ == EnumStateMachine::DTLS_DONE) {
    SET_PC_STATE(belongingPC_, EnumStateMachine::GOT_RTP);

    // notify others I entered
    auto peerPC = belongingPC_.FindPeerPC();
    if (nullptr != peerPC) {
      peerPC->dataChannelHandler_.SendSctpDataForLable(
          "I'm coming. Give me I frame.");

      ret = peerPC->rtcpHandler_.psfb_.pli_.CreatePLISend();
      if (ret) {
        tylog("create pli ret=%d", ret);
        assert(!"why fail!");
      }
    }

    // init push handler
    const char *url = std::getenv("TY_PUSH_URL");
    if (nullptr != url) {
      tylog("push url=%s", url);

      RtmpHandler &rtmpPusher = *new RtmpHandler(this->belongingPC_);  // FIXME

      ret = this->belongingPC_.pushHandler_.InitPushHandler(
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

      RtmpPuller &rtmpPuller = *new RtmpPuller(this->belongingPC_);  // FIXME

      ret = this->belongingPC_.pullHandler_.InitPullHandler(
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
  }

  assert(belongingPC_.stateMachine_ >= EnumStateMachine::GOT_RTP);

  std::string mediaType =
      reinterpret_cast<const RtpHeader *>(vBufReceive.data())
          ->ComputeMediaType();
  tylog("receive %s", mediaType.data());

  // should refactor if else for media type

  if (mediaType == kMediaTypeRtcp) {
    ret = belongingPC_.srtpHandler_.UnprotectRtcp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("unprotect RTCP fail, ret=%d", ret);

      return ret;
    }
    DumpRecvPacket(vBufReceive);

    ret = belongingPC_.rtcpHandler_.HandleRtcpPacket(vBufReceive);
    if (ret) {
      tylog("handleRtcpPacket fail, ret=%d", ret);

      return ret;
    }
  } else if (mediaType == kMediaTypeAudio || mediaType == kMediaTypeVideo) {
    tylog("before unprotect(may have) rtp, paddinglen=%d",
          getRtpPaddingLength(vBufReceive));
    // reuse original buffer
    // taylor consider restart svr
    ret = belongingPC_.srtpHandler_.UnprotectRtp(
        const_cast<std::vector<char> *>(&vBufReceive));
    if (ret) {
      tylog("warning: unprotect RTP (not RTCP) fail ret=%d", ret);

      return ret;
    }
    DumpRecvPacket(vBufReceive);
    tylog("after unprotect(may have), paddinglen=%d",
          getRtpPaddingLength(vBufReceive));

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
      // https://en.cppreference.com/w/cpp/utility/tuple/forward_as_tuple
      auto p = ssrcInfoMap_.emplace(
          std::piecewise_construct, std::forward_as_tuple(rtpHeader.getSSRC()),
          std::forward_as_tuple(*this, rtpHeader.getSSRC(),
                                mediaType == kMediaTypeAudio));
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

    assert(ssrcInfo.biggestCycle >= 0);

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
        if (ssrcInfo.biggestCycle == 0) {
          tylog("recv unusual packet (pull stream or ICE change)=%s.",
                rtpHeader.ToString().data());
          // web browser first seq is always AheadOf 0,
          // but ICE change, the seq is e.g. 63606 is not AheadOf 0.
          // TODO: ICE change, reserve session
          ssrcInfo.biggestSeq = itemSeq;
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

    bool valid = ssrcInfo.rtpReceiver.rtp_valid_packet_in_sequence(
        &ssrcInfo.rtpReceiver.rtpStats_,
        reinterpret_cast<const RtpHeader *>(rtpBizPacket.rtpRawPacket.data())
            ->getSeqNumber());
    if (!valid) {
      tylog("valid check false");
      return -2396;
    }

    // https://datatracker.ietf.org/doc/html/rfc3550#appendix-A.3
    // The number of packets received is simply the count of packets as they
    // arrive, including any late or duplicate packets.
    // So count before check old.
    ssrcInfo.rtpReceiver.CountStatistics(rtpBizPacket);

    if (rtpBizPacket.GetPowerSeq() <=
        ssrcInfo.rtpReceiver.lastPoppedPowerSeq_) {
      tylog("recv old current=%s <= lastPop=%s, ignore current pkt.",
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
      if (this->belongingPC_.pushHandler_.InitSucc()) {
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
      "{upAudioSSRC=%u, upVideoSSRC=%u, ssrcMapSize=%zu}", upAudioSSRC,
      upVideoSSRC, ssrcInfoMap_.size());
}

}  // namespace tywebrtc
