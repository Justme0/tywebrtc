// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/transport/receiver/recv_statistics.h"

namespace tywebrtc {

#define RTP_SEQ_MOD (1 << 16)

// Typical values for the parameters are shown, based on a maximum
// misordering time of 2 seconds at 50 packets/second and a maximum
// dropout of 1 minute.  The dropout parameter MAX_DROPOUT should be a
// small fraction of the 16-bit sequence number space to give a
// reasonable probability that new sequence numbers after a restart will
// not fall in the acceptable range for sequence numbers from before the
// restart.
const int MAX_DROPOUT = 3000;
const int MAX_MISORDER = 100;
const int MIN_SEQUENTIAL = 2;

#ifdef USE_FFMPEG_JITTER
static void rtcp_update_jitter(RTPStatistics* s, uint32_t sent_timestamp,
                               uint32_t arrival_timestamp) {
  // Most of this is pretty straight from RFC 3550 appendix A.8
  uint32_t transit = arrival_timestamp - sent_timestamp;
  uint32_t prev_transit = s->transit;
  int32_t d = transit - prev_transit;
  // Doing the FFABS() call directly on the "transit - prev_transit"
  // expression doesn't work, since it's an unsigned expression. Doing the
  // transit calculation in unsigned is desired though, since it most
  // probably will need to wrap around.
  d = std::abs(d);
  s->transit = transit;
  if (!prev_transit) return;
  s->jitter += d - (int32_t)((s->jitter + 8) >> 4);
}
#endif

// from
// https://source.chromium.org/chromium/chromium/src/+/main:third_party/webrtc/modules/rtp_rtcp/source/receive_statistics_impl.cc;l=152
void RTPStatistics::UpdateJitter_(const RtpBizPacket& rtpBizPacket,
                                  int64_t receive_time) {
  int64_t receive_diff = receive_time - last_receive_time_;
  assert(receive_diff >= 0);

  const int payload_type_frequency =
      rtpBizPacket.GetRtpHeader().GetPayloadTypeFrequency();
  uint32_t receive_diff_rtp = (receive_diff * payload_type_frequency) / 1000;

  // data type is key, if rtp seq reach cycle
  int32_t time_diff_samples =
      receive_diff_rtp -
      (rtpBizPacket.GetRtpHeader().getTimestamp() - last_received_timestamp_);

  ReviseFrequencyAndJitter_(payload_type_frequency);

  // lib_jingle sometimes deliver crazy jumps in TS for the same stream.
  // If this happens, don't update jitter value. Use 5 secs video frequency
  // as the threshold.
  if (time_diff_samples < 5 * kVideoPayloadTypeFrequency &&
      time_diff_samples > -5 * kVideoPayloadTypeFrequency) {
    // Note we calculate in Q4 to avoid using float.
    int32_t jitter_diff_q4 = (std::abs(time_diff_samples) << 4) - jitter_q4_;
    jitter_q4_ += ((jitter_diff_q4 + 8) >> 4);
    tylog(
        "jitterQ4=%5u, freq=%d, ssrc=%u, recvDiff-sendDiff %4u-%d=%5d.",
        jitter_q4_, payload_type_frequency,
        rtpBizPacket.GetRtpHeader().getSSRC(), receive_diff_rtp,
        (rtpBizPacket.GetRtpHeader().getTimestamp() - last_received_timestamp_),
        time_diff_samples);
    // a little different from RFC(also ffmpeg)
    // s->jitter += d - (int32_t)((s->jitter + 8) >> 4);
  } else {
    tylog("shit time_diff_samples=%d not normal.", time_diff_samples);
  }
}

void RTPStatistics::ReviseFrequencyAndJitter_(int payload_type_frequency) {
  if (payload_type_frequency == last_payload_type_frequency_) {
    return;
  }

  tylog("freqence change %d->%d, this=%s.", last_payload_type_frequency_,
        payload_type_frequency, ToString().data());

  if (payload_type_frequency != 0) {
    if (last_payload_type_frequency_ != 0) {
      // Value in "jitter_q4_" variable is a number of samples.
      // I.e. jitter = timestamp (s) * frequency (Hz).
      // Since the frequency has changed we have to update the number of samples
      // accordingly. The new value should rely on a new frequency.

      // If we don't do such procedure we end up with the number of samples that
      // cannot be converted into TimeDelta correctly
      // (i.e. jitter = jitter_q4_ >> 4 / payload_type_frequency).
      // In such case, the number of samples has a "mix".

      // Doing so we pretend that everything prior and including the current
      // packet were computed on packet's frequency.
      jitter_q4_ = static_cast<int>(static_cast<uint64_t>(jitter_q4_) *
                                    payload_type_frequency /
                                    last_payload_type_frequency_);
      tylog("statics=%s.", ToString().data());
    }
    // If last_payload_type_frequency_ is not present, the jitter_q4_
    // variable has its initial value.

    // Keep last_payload_type_frequency_ up to date and non-zero (set).
    last_payload_type_frequency_ = payload_type_frequency;
  }
}

void RTPStatistics::CountStatistics(const RtpBizPacket& rtpBizPacket) {
  // should add size before decrypt?
  octet_count += rtpBizPacket.rtpRawPacket.size();

  /* TODO: I think this is way too often; RFC 1889 has algorithm for this */
  /* XXX: MPEG pts hardcoded. RTCP send every 0.5 seconds */

  /* RTCP packets use 0.5% of the bandwidth */
  const int RTCP_TX_RATIO_NUM = 5;
  const int RTCP_TX_RATIO_DEN = 1000;
  int rtcp_bytes = ((octet_count - last_octet_count) * RTCP_TX_RATIO_NUM) /
                   RTCP_TX_RATIO_DEN;

  // mmu_man: that's enough for me... VLC sends much less btw !?
  rtcp_bytes /= 50;
  if (rtcp_bytes < 28) {
    // not send rtcp
  };
  // TODO: move rtcp send timer here

  const int64_t now_ms = g_now_ms;
  // If new time stamp and more than one in-order packet received, calculate
  // new jitter statistics.
  // So same video frame only update once.
  if (rtpBizPacket.GetRtpHeader().getTimestamp() != last_received_timestamp_ &&
      received > 1) {
    UpdateJitter_(rtpBizPacket, now_ms);
  } else {
    tylog("not upateJitter, this=%s.", ToString().data());
  }

  last_received_timestamp_ = rtpBizPacket.GetRtpHeader().getTimestamp();
  last_receive_time_ = now_ms;
}

/*
 * Called whenever there is a large jump in sequence numbers,
 * or when they get out of probation...
 */
void RTPStatistics::rtp_init_sequence(uint16_t seq) {
  max_seq = seq;
  cycles = 0;
  base_seq = seq;
  bad_seq = RTP_SEQ_MOD + 1;
  // not clear probation
  received = 0;
  expected_prior = 0;
  received_prior = 0;
  // s->jitter = 0;
  transit = 0;
}

int RTPStatistics::rtp_valid_packet_in_sequence(uint16_t seq) {
  uint16_t udelta = seq - max_seq;

  /* source not valid until MIN_SEQUENTIAL packets with sequence
   * seq. numbers have been received */
  if (probation) {
    if (seq == max_seq + 1) {
      probation--;
      max_seq = seq;
      if (probation == 0) {
        tylog("recv seq=%u, udelta=%u, to call init_sequence, receiver=%s.",
              seq, udelta, ToString().data());
        rtp_init_sequence(seq);
        received++;
        return 0;
      }
    } else {
      probation = MIN_SEQUENTIAL - 1;
      max_seq = seq;
    }
  } else if (udelta < MAX_DROPOUT) {
    // in order, with permissible gap
    if (seq < max_seq) {
      // sequence number wrapped; count another 64k cycles
      cycles += RTP_SEQ_MOD;
    }
    max_seq = seq;
  } else if (udelta <= RTP_SEQ_MOD - MAX_MISORDER) {
    // OPT: not consider NACK recv old seq?
    // sequence made a large jump...
    tylog("seq=%u, udelta[%u]<=%d, recver=%s.", seq, udelta,
          RTP_SEQ_MOD - MAX_MISORDER, ToString().data());
    if (seq == bad_seq) {
      /* two sequential packets -- assume that the other side
       * restarted without telling us; just resync. */
      rtp_init_sequence(seq);
    } else {
      bad_seq = (seq + 1) & (RTP_SEQ_MOD - 1);
      return -12859;
    }
  } else {
    // duplicate or reordered packet...
  }
  received++;
  return 0;
}

}  // namespace tywebrtc