// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_CONGEST_CONTROL_GOOGLE_CC_REMB_REMB_ABS_SEND_TIME_H_
#define SRC_CONGEST_CONTROL_GOOGLE_CC_REMB_REMB_ABS_SEND_TIME_H_

#include "src/rtp/rtp_parser.h"

namespace tywebrtc {

using TimeDelta = int64_t;  // ms

class RemoteBitrateEstimatorAbsSendTime {
 public:
  RemoteBitrateEstimatorAbsSendTime() = delete;
  RemoteBitrateEstimatorAbsSendTime(const RemoteBitrateEstimatorAbsSendTime&) =
      delete;
  RemoteBitrateEstimatorAbsSendTime& operator=(
      const RemoteBitrateEstimatorAbsSendTime&) = delete;

  void IncomingPacket(const std::vector<char>& vBufReceive);
  /*
  TimeDelta Process();
  void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms);
  void RemoveStream(uint32_t ssrc);
  // DataRate LatestEstimate() const;

 private:
  struct Probe {
    Probe(int64_t send_time, int64_t recv_time, DataSize payload_size)
        : send_time(send_time),
          recv_time(recv_time),
          payload_size(payload_size) {}

    int64_t send_time;
    int64_t recv_time;
    DataSize payload_size;
  };

  struct Cluster {
    DataRate SendBitrate() const { return mean_size / send_mean; }
    DataRate RecvBitrate() const { return mean_size / recv_mean; }

    TimeDelta send_mean = 0;
    TimeDelta recv_mean = 0;
    // TODO(holmer): Add some variance metric as well?
    DataSize mean_size = DataSize::Zero();
    int count = 0;
    int num_above_min_delta = 0;
  };

  enum class ProbeResult { kBitrateUpdated, kNoUpdate };

  // static bool IsWithinClusterBounds(TimeDelta send_delta, const Cluster&
  // cluster_aggregate);

  // static void MaybeAddCluster(const Cluster& cluster_aggregate,
  // std::list<Cluster>& clusters);

  //  std::list<Cluster> ComputeClusters() const;

  const Cluster* FindBestProbe(const std::list<Cluster>& clusters) const;

  // Returns true if a probe which changed the estimate was detected.
  ProbeResult ProcessClusters(int64_t now);

  bool IsBitrateImproving(DataRate probe_bitrate) const;
  */

  void TimeoutStreams(int64_t now);

  /*
    std::unique_ptr<InterArrival> inter_arrival_;
    std::unique_ptr<OveruseEstimator> estimator_;
    OveruseDetector detector_;
    BitrateTracker incoming_bitrate_{kBitrateWindow};
    */
  bool incoming_bitrate_initialized_ = false;
  // std::list<Probe> probes_;
  size_t total_probes_received_ = 0;
  int64_t first_packet_time_ = 0;
  int64_t last_update_ = 0;
  bool uma_recorded_ = false;

  std::map<uint32_t, int64_t> ssrcs_;
  // AimdRateControl remote_rate_;
};

}  // namespace tywebrtc

#endif  // SRC_CONGEST_CONTROL_GOOGLE_CC_REMB_REMB_ABS_SEND_TIME_H_