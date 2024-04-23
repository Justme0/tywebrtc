// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_RTCP_RTCP_HANDLER_H_
#define SRC_RTP_RTCP_RTCP_HANDLER_H_

#include <vector>

#include "src/rtp/rtcp/rtcp_packet/payload_spec_fb/rtcp_pli.h"
#include "src/rtp/rtcp/rtcp_packet/rtcp_extended_reports.h"
#include "src/rtp/rtcp/rtcp_packet/rtcp_receiver_report.h"
#include "src/rtp/rtcp/rtcp_packet/rtcp_sender_report.h"
#include "src/rtp/rtcp/rtcp_packet/rtp_fb/rtcp_nack.h"

namespace tywebrtc {

class PeerConnection;

class RtcpHandler {
 public:
  explicit RtcpHandler(PeerConnection& pc);

  int HandleRtcpPacket(const std::vector<char>& vBufReceive);

  std::string ToString() const;

 public:  // should be private
  RtcpSenderReport senderReport_;
  RtcpReceiverReport receiverReport_;
  RtcpNack nack_;
  RtcpPLI pli_;
  RtcpExtendedReports extendedReport_;

  PeerConnection& belongingPeerConnection_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_HANDLER_H_