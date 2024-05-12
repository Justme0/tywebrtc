// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_PACK_UNPACK_H264_TO_RTP_H_
#define SRC_RTP_PACK_UNPACK_H264_TO_RTP_H_

#include <cstdint>
#include <vector>

#include "src/rtp/rtp_parser.h"

namespace tywebrtc {

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

}  // namespace tywebrtc

#endif  // SRC_RTP_PACK_UNPACK_H264_TO_RTP_H_