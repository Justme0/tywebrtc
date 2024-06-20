// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_TRANSPORT_RECEIVER_RECEIVER_H_
#define SRC_TRANSPORT_RECEIVER_RECEIVER_H_

#include <set>
#include <vector>

#include "src/rtp/rtp_parser.h"
#include "src/timer/timer.h"
#include "src/transport/receiver/recv_statistics.h"

namespace tywebrtc {

class SSRCInfo;

const PowerSeqT kShitRecvPowerSeqInitValue = -1;

class RtpReceiver {
 public:
  explicit RtpReceiver(SSRCInfo& ssrcInfo);

  void PushToJitter(RtpBizPacket&& rtpBizPacket);
  std::vector<RtpBizPacket> PopOrderedPackets();
  int GetJitterSize() const;

  std::string ToString() const;

 public:
  SSRCInfo& belongingSSRCInfo_;

  // must be ordered, cannot be hashmap
  std::map<PowerSeqT, RtpBizPacket> jitterBuffer_;

  PowerSeqT lastPoppedPowerSeq_ = kShitRecvPowerSeqInitValue;

  bool is_add_rr_timer_{};
  ReceiverReportTimer receiverReportTimer_;

  bool is_add_pli_timer_{};
  PLITimer pliTimer_;

  mutable RTPStatistics rtpStats_{};
};

}  // namespace tywebrtc

#endif  // SRC_TRANSPORT_RECEIVER_RECEIVER_H_