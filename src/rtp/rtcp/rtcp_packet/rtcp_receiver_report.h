// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_RTCP_RTCP_PACKET_RTCP_RECEIVER_REPORT_H_
#define SRC_RTP_RTCP_RTCP_PACKET_RTCP_RECEIVER_REPORT_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace tywebrtc {

class RtcpHandler;
class RtcpHeader;

struct RrPkgInfo {
  uint64_t svrTimeMS;
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

class RtcpReceiverReport {
 public:
  explicit RtcpReceiverReport(RtcpHandler &belongingRtcpHandler);

  int HandleReceiverReport(const RtcpHeader &chead);

  int CreateReceiverReport(std::vector<char> *io_rtcpBin);

 private:
  RtcpHandler &belongingRtcpHandler_;
  std::unordered_map<uint32_t, RrPkgInfo> ssrcRRInfo;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PACKET_RTCP_RECEIVER_REPORT_H_