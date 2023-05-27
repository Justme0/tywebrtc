#ifndef RTP_RTP_HANDLER_H_
#define RTP_RTP_HANDLER_H_

#include <unordered_map>
#include <vector>

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

  int HandleRtpPacket(const std::vector<char> &vBufReceive);
  // int GetUpAudioSSRC(uint32_t& ssrc) const;
  // int GetUpVideoSSRC(uint32_t& ssrc) const;
  int DumpPacket(const std::vector<char> &packet, H264Unpacketizer &unpacker);

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

 private:
  SrsAudioTranscoder audioTranscoder_;
  RtpDepacketizerVp8 videoTranscoder_;
};

#endif  // RTP_RTP_HANDLER_H_
