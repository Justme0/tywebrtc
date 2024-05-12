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

class SSRCInfo {
 public:
  explicit SSRCInfo(RtpHandler &belongingRtpHandler);

  // OPT: use a fec handler
  std::vector<std::vector<char>> EncodeFec(
      uint32_t thisSSRC, const std::vector<RtpBizPacket> &rtpBizPackets);

  std::string ToString() const;

  // why define ssrc's unpacketizer: save unpacked frame e.g. FU-A
  // audio don't use h264Unpacketizer
  H264Unpacketizer h264Unpacketizer;
  H264Packetizer h264Packetizer;
  AudioPacketizer audioPacketizer;

  RtpReceiver rtpReceiver;
  RtpSender rtpSender;

  // received biggest pkt
  uint16_t biggestSeq = 0;
  int64_t biggestCycle = 0;

  RtpHandler &belongingRtpHandler;
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
  PeerConnection &belongingPeerConnection_;

  // OPT: should assign from SDP
  uint32_t upAudioSSRC = 0;
  uint32_t upVideoSSRC = 0;

  std::unordered_map<uint32_t, SSRCInfo> ssrcInfoMap_;

 public:  // tmp
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
