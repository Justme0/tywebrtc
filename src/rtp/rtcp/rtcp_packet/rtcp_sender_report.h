// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_RTCP_RTCP_PACKET_RTCP_SENDER_REPORT_H_
#define SRC_RTP_RTCP_RTCP_PACKET_RTCP_SENDER_REPORT_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace tywebrtc {

class RtcpHandler;
class RtcpHeader;

struct SrPkgInfo {
  uint64_t svrTimeMS{};
  uint32_t SRCount{};
  uint32_t SSRC{};
  uint32_t blockCount{};
  uint32_t srLen{};
  uint64_t NTPTimeStamps{};
  uint32_t RTPTimeStamps{};
  uint32_t sentPkgs{};
  uint32_t sentOctets{};
};

class RtcpSenderReport {
 public:
  explicit RtcpSenderReport(RtcpHandler &belongingRtcpHandler);

  int HandleSenderReport(const RtcpHeader &chead);

  int CreateSenderReport(std::vector<char> *io_rtcpBin);

 private:
  RtcpHandler &belongingRtcpHandler_;
  std::unordered_map<uint32_t, SrPkgInfo> ssrcSRInfo;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PACKET_RTCP_SENDER_REPORT_H_