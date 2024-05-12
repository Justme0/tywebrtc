// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_RTCP_RTCP_PACKET_RTP_FB_RTCP_NACK_H_
#define SRC_RTP_RTCP_RTCP_PACKET_RTP_FB_RTCP_NACK_H_

#include <cstdint>
#include <set>
#include <vector>

#include "src/rtp/rtcp/rtcp_parser.h"

namespace tywebrtc {

class RtcpHandler;
class RtcpRtpFeedback;

class RtcpNack {
 public:
  explicit RtcpNack(RtcpRtpFeedback &belongingRtpfb);

  int HandleNack(const RtcpHeader &chead);

  int CreateNackSend(const std::set<int> &lostSeqs, uint32_t localSSRC,
                     uint32_t remoteSSRC);

 private:
  int SendReqNackPkt_(const std::vector<uint16_t> &seqVect, uint32_t sourceSSRC,
                      std::vector<uint16_t> &failedSeqs);

  int SerializeNackSend_(const std::vector<NackBlock> &nackBlokVect,
                         uint32_t sinkSSRC, uint32_t soucreSSRC);

 private:
  RtcpRtpFeedback &belongingRtpfb_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PACKET_RTP_FB_RTCP_NACK_H_