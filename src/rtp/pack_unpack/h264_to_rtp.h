#ifndef RTP_PACK_UNPACK_H264_TO_RTP_H_
#define RTP_PACK_UNPACK_H264_TO_RTP_H_

#include <cstdint>
#include <vector>

#include "src/rtp/rtp_parser.h"

enum enVideoRtpRfcMode {
  kH264Rfc3984Mode0 = 1,  // 模式0 Single NAL Unit
  kH264Rfc3984Mode1,      // 模式1 支持Single NAL，FU-A，STAP-A
  kH264Rfc3984Mode2,      // 模式2 支持STAP-B,MTAP16,MTAP24,FU-A,FU-B
  kH264Rfc3984ModeButt
};

class H264Packetizer {
 public:
  int Packetize(const std::vector<char> &stream, uint32_t timestamp,
                const std::vector<std::shared_ptr<Extension>> &extensions,
                std::vector<RtpBizPacket> &rtpBizPackets);

  // also used for FEC encoder
  PowerSeqT GeneratePowerSequence() { return ++powerSequence_; }

 private:
  int PacketStapA(const uint32_t timestamp,
                  const std::vector<std::shared_ptr<Extension>> &extensions,
                  std::vector<RtpBizPacket> &rtpBizPackets);

  int PacketSingleNalu(
      const char *frame, const int len, const uint32_t timestamp,
      const std::vector<std::shared_ptr<Extension>> &extensions,
      std::vector<RtpBizPacket> &rtpBizPackets);

  int PacketFuA(const char *frame, const int len, const uint32_t timestamp,
                const std::vector<std::shared_ptr<Extension>> &extensions,
                std::vector<RtpBizPacket> &rtpBizPackets);

 private:
  uint8_t payload_type_ = kDownlinkH264PayloadType;
  uint32_t ssrc_ = kDownlinkVideoSsrc;
  PowerSeqT powerSequence_ = 0;  // last packet seq
  std::string sps_;
  std::string pps_;
  // int mtu_size_byte_ = 1200;  // should probe using PMTU
};

#endif  // RTP_PACK_UNPACK_H264_TO_RTP_H_