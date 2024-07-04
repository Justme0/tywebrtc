// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/congest_control/google_cc/remb/remb_abs_send_time.h"

namespace tywebrtc {
namespace {

[[maybe_unused]] constexpr TimeDelta kMinClusterDelta = 1;
[[maybe_unused]] constexpr TimeDelta kInitialProbingInterval = 2000;
[[maybe_unused]] constexpr int kTimestampGroupLengthMs = 5;
constexpr int kAbsSendTimeInterArrivalUpshift = 8;

constexpr int kInterArrivalShift =
    kAbsSendTimeFraction + kAbsSendTimeInterArrivalUpshift;
[[maybe_unused]] constexpr int kMinClusterSize = 4;
[[maybe_unused]] constexpr int kMaxProbePackets = 15;
[[maybe_unused]] constexpr int kExpectedNumberOfProbes = 3;
constexpr double kTimestampToMs =
    1000.0 / static_cast<double>(1 << kInterArrivalShift);

}  // namespace

void RemoteBitrateEstimatorAbsSendTime::TimeoutStreams(int64_t now) {
  for (auto it = ssrcs_.begin(); it != ssrcs_.end();) {
    if (now - it->second > kStreamTimeOut) {
      ssrcs_.erase(it++);
    } else {
      ++it;
    }
  }
  if (ssrcs_.empty()) {
    // We can't update the estimate if we don't have any active streams.
    // inter_arrival_ = std::make_unique<InterArrival>( (kTimestampGroupLengthMs
    // << kInterArrivalShift) / 1000, kTimestampToMs);
    // estimator_ = std::make_unique<OveruseEstimator>();
    // We deliberately don't reset the first_packet_time_ms_ here for now since
    // we only probe for bandwidth in the beginning of a call right now.
  }
}

void RemoteBitrateEstimatorAbsSendTime::IncomingPacket(
    const std::vector<char>& vBufReceive) {
  const RtpHeader& rtp =
      *reinterpret_cast<const RtpHeader*>(vBufReceive.data());

  uint32_t send_time_24bits = 0;
  // todo: get ext abs time
  // if (!rtp_packet.GetExtension<AbsoluteSendTime>(&send_time_24bits)) {
  //   tylog( "RemoteBitrateEstimatorAbsSendTimeImpl: Incoming packet is missing
  //   absolute send time extension!");
  //   assert(!"tmp use assert");
  //   return;
  // }

  [[maybe_unused]] int64_t arrival_time = g_now_ms;

  // include padding
  [[maybe_unused]] int payload_size =
      vBufReceive.size() - rtp.getHeaderLength();

  if (!uma_recorded_) {
    tylog("first uma_recorded");
    uma_recorded_ = true;
  }
  // Shift up send time to use the full 32 bits that inter_arrival works with,
  // so wrapping works properly.
  uint32_t timestamp = send_time_24bits << kAbsSendTimeInterArrivalUpshift;
  [[maybe_unused]] int64_t send_time = timestamp * kTimestampToMs;

  // TODO(holmer): SSRCs are only needed for REMB, should be broken out from
  // here.

  // Check if incoming bitrate estimate is valid, and if it needs to be reset.
  /*
  absl::optional<DataRate> incoming_bitrate =
      incoming_bitrate_.Rate(arrival_time);
  if (incoming_bitrate) {
    incoming_bitrate_initialized_ = true;
  } else if (incoming_bitrate_initialized_) {
    // Incoming bitrate had a previous valid value, but now not enough data
    // point are left within the current window. Reset incoming bitrate
    // estimator so that the window size will only contain new data points.
    incoming_bitrate_.Reset();
    incoming_bitrate_initialized_ = false;
  }
  incoming_bitrate_.Update(payload_size, arrival_time);

  if (first_packet_time_ == 0) {
    first_packet_time_ = g_now_ms;
  }

  uint32_t ts_delta = 0;
  int64_t t_delta = 0;
  int size_delta = 0;
  bool update_estimate = false;
  DataRate target_bitrate = DataRate::Zero();

  TimeoutStreams(g_now_ms);
  assert(inter_arrival_);
  assert(estimator_);
  ssrcs_.insert_or_assign(rtp_packet.Ssrc(), g_now_ms);

  // For now only try to detect probes while we don't have a valid estimate.
  // We currently assume that only packets larger than 200 bytes are paced by
  // the sender.
  static constexpr int kMinProbePacketSize = 200;
  if (payload_size > kMinProbePacketSize &&
      (!remote_rate_.ValidEstimate() ||
       g_now_ms - first_packet_time_ < kInitialProbingInterval)) {
    // TODO(holmer): Use a map instead to get correct order?
    if (total_probes_received_ < kMaxProbePackets) {
      TimeDelta send_delta = -1;
      TimeDelta recv_delta = -1;
      if (!probes_.empty()) {
        send_delta = send_time - probes_.back().send_time;
        recv_delta = arrival_time - probes_.back().recv_time;
      }
      RTC_LOG(LS_INFO) << "Probe packet received: send time=" << send_time.ms()
                       << " ms, recv time=" << arrival_time.ms()
                       << " ms, send delta=" << send_delta.ms()
                       << " ms, recv delta=" << recv_delta.ms() << " ms.";
    }
    probes_.emplace_back(send_time, arrival_time, payload_size);
    ++total_probes_received_;
    // Make sure that a probe which updated the bitrate immediately has an
    // effect by calling the OnReceiveBitrateChanged callback.
    if (ProcessClusters(g_now_ms) == ProbeResult::kBitrateUpdated)
      update_estimate = true;
  }
  if (inter_arrival_->ComputeDeltas(timestamp, arrival_time.ms(), g_now_ms,
                                    payload_size, &ts_delta, &t_delta,
                                    &size_delta)) {
    double ts_delta_ms = (1000.0 * ts_delta) / (1 << kInterArrivalShift);
    estimator_->Update(t_delta, ts_delta_ms, size_delta, detector_.State(),
                       arrival_time.ms());
    detector_.Detect(estimator_->offset(), ts_delta_ms,
                     estimator_->num_of_deltas(), arrival_time.ms());
  }

  if (!update_estimate) {
    // Check if it's time for a periodic update or if we should update because
    // of an over-use.
    if (last_update_.IsInfinite() ||
        g_now_ms - last_update_.ms() >
            remote_rate_.GetFeedbackInterval().ms()) {
      update_estimate = true;
    } else if (detector_.State() == BandwidthUsage::kBwOverusing) {
      absl::optional<DataRate> incoming_rate =
          incoming_bitrate_.Rate(arrival_time);
      if (incoming_rate.has_value() &&
          remote_rate_.TimeToReduceFurther(g_now_ms, *incoming_rate)) {
        update_estimate = true;
      }
    }
  }

  if (update_estimate) {
    // The first overuse should immediately trigger a new estimate.
    // We also have to update the estimate immediately if we are overusing
    // and the target bitrate is too high compared to what we are receiving.
    const RateControlInput input(detector_.State(),
                                 incoming_bitrate_.Rate(arrival_time));
    target_bitrate = remote_rate_.Update(input, g_now_ms);
    update_estimate = remote_rate_.ValidEstimate();
  }

  if (update_estimate) {
    last_update_ = g_now_ms;
    // observer_->OnReceiveBitrateChanged(Keys(ssrcs_),
    // target_bitrate.bps<uint32_t>());
  }
  */
}

}  // namespace tywebrtc