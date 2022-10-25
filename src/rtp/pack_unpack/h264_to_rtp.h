#ifndef RTP_PACK_UNPACK_H264_TO_RTP_H_
#define RTP_PACK_UNPACK_H264_TO_RTP_H_

#include <cassert>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

enum enVideoRtpRfcMode {
  kH264Rfc3984Mode0 = 1,  // 模式0 Single NAL Unit
  kH264Rfc3984Mode1,      // 模式1 支持Single NAL，FU-A，STAP-A
  kH264Rfc3984Mode2,      // 模式2 支持STAP-B,MTAP16,MTAP24,FU-A,FU-B
  kH264Rfc3984ModeButt
};

class H264Packetizer {
 public:
  int Packetize(const uint8_t *data, uint32_t length, uint32_t timestamp,
                const RtpHeader::ExtensionList &extensions,
                std::vector<std::unique_ptr<RtpPacket>> &rtp_packets) override;

 private:
  int PacketFuA(const uint8_t *frame, const int len, const uint32_t timestamp,
                const RtpHeader::ExtensionList &extensions,
                std::vector<std::unique_ptr<RtpPacket>> &packets);
  int PacketSingleNalu(const uint8_t *frame, const int len,
                       const uint32_t timestamp,
                       const RtpHeader::ExtensionList &extensions,
                       std::vector<std::unique_ptr<RtpPacket>> &packets);
  int PacketStapA(const uint32_t timestamp,
                  const RtpHeader::ExtensionList &extensions,
                  std::vector<std::unique_ptr<RtpPacket>> &packets);

 private:
  std::string sps_;
  std::string pps_;
};

#endif  // RTP_PACK_UNPACK_H264_TO_RTP_H_