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

namespace tywebrtc {

class SSRCInfo;

const PowerSeqT kShitRecvPowerSeqInitValue = -1;

// these statistics are used for rtcp receiver reports...
struct RTPStatistics {
  uint16_t max_seq;         ///< highest sequence number seen
  uint32_t cycles;          ///< shifted count of sequence number cycles
  uint32_t base_seq;        ///< base sequence number
  uint32_t bad_seq;         ///< last bad sequence number + 1
  int probation = 1;        ///< sequence packets till source is valid
  uint32_t received;        ///< packets received
  uint32_t expected_prior;  ///< packets expected in last interval
  uint32_t received_prior;  ///< packets received in last interval
  uint32_t transit;         ///< relative transit time for previous packet
  uint32_t jitter;          ///< estimated jitter.

  // not in RFC sample code
  unsigned int octet_count;
  unsigned int last_octet_count;

  std::string ToString() const {
    return tylib::format_string(
        "{max_seq=%u, cycles=%u, base_seq=%u, bad_seq=%u, probation=%d, "
        "received=%u, expected_prior=%u, received_prior=%u, transit=%u, "
        "jitter=%u, octet_count=%u, last_octet_count=%u}",
        max_seq, cycles, base_seq, bad_seq, probation, received, expected_prior,
        received_prior, transit, jitter, octet_count, last_octet_count);
  }
};

class RtpReceiver {
 public:
  explicit RtpReceiver(SSRCInfo& ssrcInfo);

  bool rtp_valid_packet_in_sequence(RTPStatistics* s, uint16_t seq);
  void CountStatistics(const RtpBizPacket& rtpBizPacket);
  void PushToJitter(RtpBizPacket&& rtpBizPacket);
  std::vector<RtpBizPacket> PopOrderedPackets();
  int GetJitterSize() const;

  std::string ToString() const;

 public:
  SSRCInfo& belongingSSRCInfo_;

  RTPStatistics rtpStats_{};

  // must be ordered, cannot be hashmap
  std::map<PowerSeqT, RtpBizPacket> jitterBuffer_;

  PowerSeqT lastPoppedPowerSeq_ = kShitRecvPowerSeqInitValue;
};

}  // namespace tywebrtc

#endif  //   SRC_TRANSPORT_RECEIVER_RECEIVER_H_