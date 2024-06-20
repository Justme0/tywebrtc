// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_RTP_HANDLER_H_
#define SRC_RTP_RTP_HANDLER_H_

#include <unordered_map>
#include <vector>

extern "C" {
#include "libavformat/avformat.h"
}

#include "src/codec/audio_codec.h"
#include "src/push/push_handler.h"
#include "src/rtmp/rtmp_handler.h"
#include "src/rtmp/rtmp_pull.h"
#include "src/rtp/pack_unpack/audio_to_rtp.h"
#include "src/rtp/pack_unpack/h264_to_rtp.h"
#include "src/rtp/pack_unpack/rtp_to_h264.h"
#include "src/rtp/pack_unpack/rtp_to_vp8.h"
#include "src/transport/receiver/receiver.h"
#include "src/transport/sender/sender.h"

namespace tywebrtc {

// don't include whole head file avoid recycle reference
class PeerConnection;
class RtpHandler;

struct RrPkgInfo {
  int64_t recvMs;
  uint32_t RRCount;
  uint32_t sinkSSRC;
  uint32_t sourceSSRC;
  uint8_t fractionLost;
  int32_t lostPkgNum;
  uint32_t extendedSeq;
  uint32_t jitter;
  uint32_t lastSr;
  uint32_t delaySinceLast;

  // tostring
};

struct RrtrPkgInfo {
  int64_t recvMs{};
  uint64_t rrtrNtp{};
};

struct SrPkgInfo {
  int64_t recvMs{};
  uint32_t SRCount{};
  uint32_t SSRC{};
  uint32_t blockCount{};
  uint32_t srLen{};
  uint64_t NTPTimeStamps{};
  uint32_t RTPTimeStamps{};
  uint32_t sentPkgs{};
  uint32_t sentOctets{};
};

class SSRCInfo {
 public:
  explicit SSRCInfo(RtpHandler &belongingRtpHandler, uint32_t ssrc,
                    bool is_audio);

  // OPT: use a fec handler
  std::vector<std::vector<char>> EncodeFec(
      const std::vector<RtpBizPacket> &rtpBizPackets);

  std::string ToString() const;

  // should be private
 public:
  // why define ssrc's unpacketizer: save unpacked frame e.g. FU-A
  // audio don't use h264Unpacketizer.
  // why cannot use union, to fix:
  // destructor of 'SSRCInfo' is implicitly deleted because variant field
  // 'h264Unpacketizer' has a non-trivial destructor
  H264Unpacketizer h264Unpacketizer;
  H264Packetizer h264Packetizer;
  AudioPacketizer audioPacketizer;

  RtpReceiver rtpReceiver;
  RtpSender rtpSender;

  uint16_t biggestSeq = 0;
  int64_t biggestCycle = 0;

  RtpHandler &belongingRtpHandler;

  RrPkgInfo rrInfo_{};
  SrPkgInfo srInfo_{};

  const uint32_t ssrc_key_{};
  const bool is_audio_{};
};

class RtpHandler {
 public:
  explicit RtpHandler(PeerConnection &pc);
  ~RtpHandler();

  int HandleRtpPacket(const std::vector<char> &vBufReceive);
  // int GetUpAudioSSRC(uint32_t& ssrc) const;
  // int GetUpVideoSSRC(uint32_t& ssrc) const;
  int DumpPacket(const std::vector<char> &packet, H264Unpacketizer &unpacker);
  int WriteWebmFile(const std::string &frame, uint32_t rtpTs,
                    const std::string &mediaType, bool bKeyFrame);

  std::string ToString() const;

 private:
  // int HandleRtcpPacket_(const std::vector<char> &vBufReceive);
  int SendToPeer_(RtpBizPacket &rtpBizPacket);

 public:
  PeerConnection &belongingPC_;

  // OPT: should assign from SDP
  uint32_t upAudioSSRC = 0;
  uint32_t upVideoSSRC = 0;

  // no related with SSRC, receiver's SSRC no use
  RrtrPkgInfo rrtrInfo_{};

  std::unordered_map<uint32_t, SSRCInfo> ssrcInfoMap_;

  // downlink may have multiple
  SrsAudioTranscoder audioTranscoderDownlink_;

  // OPT: use struct
  // write uplink stream to WebM file.
  // FIX: close in destructor
  // Write trailer
  // av_write_trailer(formatContext);
  // Cleanup
  // avio_close(formatContext->pb);
  // avformat_free_context(formatContext);
  AVFormatContext *uplinkFileCtx_ = nullptr;
  int audioStreamIndex_ = 0;
  int videoStreamIndex_ = 0;

  // FIX: only support 2^32/90000/3600=13 hours,
  // should use extend RTP timestamp in 64bit
  // what if RTP TS roll back?
  uint32_t firstRtpVideoTs_ = 0;
  uint32_t firstRtpAudioTs_ = 0;

 private:
  // uplink
  SrsAudioTranscoder audioTranscoder_;
  RtpDepacketizerVp8 videoTranscoder_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTP_HANDLER_H_
