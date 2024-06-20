// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_TRANSPORT_RECEIVER_RECV_STATISTICS_H_
#define SRC_TRANSPORT_RECEIVER_RECV_STATISTICS_H_

#include "src/rtp/rtp_parser.h"

#include "tylib/string/format_string.h"

namespace tywebrtc {

// these statistics are used for rtcp receiver reports...
struct RTPStatistics {
  uint16_t max_seq;         ///< highest sequence number seen
  uint32_t cycles;          ///< shifted count of sequence number cycles
  uint32_t base_seq;        ///< base sequence number
  uint32_t bad_seq;         ///< last bad sequence number + 1
  int probation;            ///< sequence packets till source is valid
  uint32_t received;        ///< packets received
  uint32_t expected_prior;  ///< packets expected in last interval
  uint32_t received_prior;  ///< packets received in last interval
  uint32_t transit;         ///< relative transit time for previous packet
  // use jitter_q4_ from WebRTC
  // uint32_t jitter;          ///< estimated jitter.

  // not in RFC sample code
  unsigned int octet_count;
  unsigned int last_octet_count;

  // RR jitter
  int64_t last_receive_time_;
  uint32_t last_received_timestamp_;
  // Stats on received RTP packets.
  uint32_t jitter_q4_;
  // The sample frequency of the last received packet.
  int last_payload_type_frequency_;

  void CountStatistics(const RtpBizPacket& rtpBizPacket);

  int rtp_valid_packet_in_sequence(uint16_t seq);

  void rtp_init_sequence(uint16_t seq);

  std::string ToString() const {
    return tylib::format_string(
        "{max_seq=%u, cycles=%u, base_seq=%u, bad_seq=%u, probation=%d, "
        "received=%u, expected_prior=%u, received_prior=%u, transit=%u, "
        "octet_count=%u, last_octet_count=%u, "
        "last_receive_time=%ld[%s], last_received_timestamp=%u, jitter_q4=%u, "
        "last_payload_type_frequency=%d}",
        max_seq, cycles, base_seq, bad_seq, probation, received, expected_prior,
        received_prior, transit, octet_count, last_octet_count,
        last_receive_time_,
        tylib::MilliSecondToLocalTimeString(last_receive_time_).data(),
        last_received_timestamp_, jitter_q4_, last_payload_type_frequency_);
  }

 private:
  void UpdateJitter_(const RtpBizPacket& rtpBizPacket, int64_t receive_time);

  void ReviseFrequencyAndJitter_(int payload_type_frequency);
};

}  // namespace tywebrtc

#endif  // SRC_TRANSPORT_RECEIVER_RECV_STATISTICS_H_