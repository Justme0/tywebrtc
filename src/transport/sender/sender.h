// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_TRANSPORT_SENDER_SENDER_H_
#define SRC_TRANSPORT_SENDER_SENDER_H_

#include <set>
#include <vector>

#include "src/rtp/rtp_parser.h"
#include "src/timer/timer.h"

namespace tywebrtc {

class SSRCInfo;

struct RTPSendStatistics {
  unsigned int packet_count{};
  unsigned int octet_count{};
};

// audio should not in pacing
class RtpSender {
 public:
  explicit RtpSender(SSRCInfo& ssrcInfo);

  void Enqueue(RtpBizPacket&& rtpBizPacket);

  std::vector<RtpBizPacket> Dequeue();

  // int GetQueueSize() const;
  const std::vector<char>* GetSeqPacket(PowerSeqT powerSeq) const;

  void SetLastRtpTime(uint32_t rtp_timestamp, uint64_t capture_time_ms) {
    last_rtp_timestamp_ = rtp_timestamp;
    last_frame_capture_time_ = capture_time_ms;
  }

  uint32_t last_rtp_timestamp() const { return last_rtp_timestamp_; }

  uint64_t last_frame_capture_time() const { return last_frame_capture_time_; }

  SSRCInfo& belongingSSRCInfo_;

 private:
  std::map<PowerSeqT, RtpBizPacket> sendQueue_{};

  // [begin, saveLast_] is already sent, saved for downlink NACK, len <=
  // kSendQueueSaveLen
  const size_t kSendQueueSaveLen = 1000;

  // must be initialized to map end(), and defined after sendQueue_, see
  // constructor
  std::map<PowerSeqT, RtpBizPacket>::const_iterator saveLast_;

  uint32_t last_rtp_timestamp_{};
  uint64_t last_frame_capture_time_{};

  bool is_add_sr_timer_{};
  SenderReportTimer senderReportTimer_;

 public:
  mutable RTPSendStatistics rtpSendStats_{};
};

}  // namespace tywebrtc

#endif  //   SRC_TRANSPORT_SENDER_SENDER_H_