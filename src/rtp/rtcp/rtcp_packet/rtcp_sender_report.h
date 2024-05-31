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

class RtcpSenderReport {
 public:
  explicit RtcpSenderReport(RtcpHandler &belongingRtcpHandler);

  int HandleSenderReport(const RtcpHeader &chead);

  int CreateSenderReport(std::vector<char> *io_rtcpBin);

 private:
  RtcpHandler &belongingRtcpHandler_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PACKET_RTCP_SENDER_REPORT_H_