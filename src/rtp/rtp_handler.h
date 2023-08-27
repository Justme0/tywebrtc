#ifndef RTP_RTP_HANDLER_H_
#define RTP_RTP_HANDLER_H_

#include <unordered_map>
#include <vector>

extern "C" {
#include "libavformat/avformat.h"
}

#include "codec/audio_codec.h"
#include "push/push_handler.h"
#include "rtmp/rtmp_handler.h"
#include "rtmp/rtmp_pull.h"
#include "rtp/pack_unpack/audio_to_rtp.h"
#include "rtp/pack_unpack/h264_to_rtp.h"
#include "rtp/pack_unpack/rtp_to_h264.h"
#include "rtp/pack_unpack/rtp_to_vp8.h"
#include "transport/receiver/receiver.h"
#include "transport/sender/sender.h"

// don't include whole head file avoid recycle reference
class PeerConnection;
class RtpHandler;

struct SSRCInfo {
  explicit SSRCInfo(RtpHandler &belongingRtpHandler);

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
  int HandleRtcpPacket_(const std::vector<char> &vBufReceive);
  int SendToPeer_(RtpBizPacket &rtpBizPacket);

 public:
  PeerConnection &belongingPeerConnection_;

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

#endif  // RTP_RTP_HANDLER_H_
