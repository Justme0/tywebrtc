#ifndef RTP_RTP_HANDLER_H_
#define RTP_RTP_HANDLER_H_

#include <unordered_map>
#include <vector>

#include "codec/audio_codec.h"
#include "rtmp/rtmp_handler.h"
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
  RtmpHandler rtmpHandler;

  std::unordered_map<uint32_t, SSRCInfo> ssrcInfoMap_;

 private:
  SrsAudioTranscoder audioTranscoder_;
  RtpDepacketizerVp8 videoTranscoder_;
};

#endif  // RTP_RTP_HANDLER_H_
