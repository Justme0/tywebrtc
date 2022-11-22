#ifndef RTP_RTCP_RTCP_HANDLER_H_
#define RTP_RTCP_RTCP_HANDLER_H_

#include <deque>
#include <map>
#include <vector>

#ifndef __USE_SINGLE_THREAD__
#include <mutex>
#endif

class PeerConnection;

const uint32_t kRtcpSenderTimeoutMs = 3000;

/*
class RTCPData {
 public:
  // using shared = std::shared_ptr<RTCPData>;

 public:
  explicit RTCPData(RTCPPacket::Type type) : type_(type) {}
  explicit RTCPData(RTCPPacket::Type type,
                    RTCPRTPFeedback::FeedbackType feedback_type)
      : type_(type), rtp_feedback_type_(feedback_type) {}
  explicit RTCPData(RTCPPacket::Type type,
                    RTCPPayloadFeedback::FeedbackType feedback_type)
      : type_(type), payload_feedback_type_(feedback_type) {}

  virtual uint32_t Serialize(char* out, int max_len) = 0;

  inline RTCPPacket::Type Type() { return type_; }
  inline RTCPRTPFeedback::FeedbackType RtpType() { return rtp_feedback_type_; }
  inline RTCPPayloadFeedback::FeedbackType PayloadType() {
    return payload_feedback_type_;
  }

 public:
  RTCPPacket::Type type_;
  RTCPRTPFeedback::FeedbackType rtp_feedback_type_;
  RTCPPayloadFeedback::FeedbackType payload_feedback_type_;
};

class RTCPSenderReportData : public RTCPData {
 public:
  using shared = std::shared_ptr<RTCPSenderReportData>;

 public:
  RTCPSenderReportData(uint32_t local_ssrc, uint32_t last_rtp_ts,
                       uint32_t total_bytes, uint32_t total_pkg_num)
      : RTCPData(RTCPPacket::SenderReport) {
    local_ssrc_ = local_ssrc;
    now_time_ms_ = g_now_ms;
    last_rtp_ts_ = last_rtp_ts;
    total_bytes_ = total_bytes;
    total_pkg_num_ = total_pkg_num;
  }

 public:
  uint32_t local_ssrc_;
  uint64_t now_time_ms_;
  uint32_t last_rtp_ts_;
  uint32_t total_bytes_;
  uint32_t total_pkg_num_;
};

class RTCPXrData : public RTCPData {
 public:
  using shared = std::shared_ptr<RTCPXrData>;

 public:
  RTCPXrData(uint32_t ssrc, ) : RTCPData(RTCPPacket::XrExtend) {
    ssrc_ = ssrc;
    ntp_ = MsToNtp(now_time_ms);
  }

 public:
  uint32_t ssrc_;
  NtpTime ntp_;
};

class RTCPReceiverReportData : public RTCPData {
 public:
  using shared = std::shared_ptr<RTCPSenderReportData>;

 public:
  RTCPReceiverReportData(uint32_t local_ssrc, uint32_t remote_ssrc,
                         uint32_t recv_pkg_num, uint32_t last_recv_pkg_num,
                         uint32_t frac_lost, uint32_t jitter, uint32_t lost, ,
                         uint32_t extended_seq, uint32_t rbe_bitrate)
      : RTCPData(RTCPPacket::ReceiverReport) {
    local_ssrc_ = local_ssrc;
    remote_ssrc_ = remote_ssrc;
    recv_pkg_num_ = recv_pkg_num;
    last_recv_pkg_num_ = last_recv_pkg_num;
    frac_lost_ = frac_lost;
    jitter_ = jitter;
    lost_ = lost;
    extended_seq_ = extended_seq;
    rbe_bitrate_ = rbe_bitrate;
    now_time_ms_ = g_now_ms;
  }

 public:
  uint32_t remote_ssrc_;
  uint32_t local_ssrc_;
  uint32_t recv_pkg_num_;
  uint32_t last_recv_pkg_num_;
  uint32_t frac_lost_;
  uint32_t jitter_;
  uint32_t lost_;
  uint32_t extended_seq_;
  uint32_t rbe_bitrate_;
  uint64_t now_time_ms_;
};

class RTCPPLIReportData : public RTCPData {
 public:
  using shared = std::shared_ptr<RTCPPLIReportData>;

 public:
  RTCPPLIReportData(uint32_t local_ssrc, uint32_t remote_ssrc)
      : RTCPData(RTCPPacket::PayloadFeedback,
                 RTCPPayloadFeedback::PictureLossIndication) {
    local_ssrc_ = local_ssrc;
    remote_ssrc_ = remote_ssrc;
  }

  uint32_t local_ssrc_;
  uint32_t remote_ssrc_;
};

class RTCPNackReportData : public RTCPData {
 public:
  using shared = std::shared_ptr<RTCPNackReportData>;

 public:
  RTCPNackReportData(uint32_t local_ssrc, uint32_t remote_ssrc,
                     std::vector<uint16_t>& seqs)
      : RTCPData(RTCPPacket::RTPFeedback, RTCPRTPFeedback::NACK) {
    local_ssrc_ = local_ssrc;
    remote_ssrc_ = remote_ssrc;
    seqs_ = seqs;
  }

  uint32_t local_ssrc_;
  uint32_t remote_ssrc_;
  std::vector<uint16_t> seqs_;
};

struct SenderReportInfo {
  uint64_t time_ms;
  uint64_t ntp_time;
};

class RTCPObserver {
 public:
  virtual ~RTCPObserver() = default;
  virtual uint32_t GetSampleRateKhz(uint32_t ssrc) = 0;
  virtual void OnReceiverReport(uint32_t local_ssrc, uint32_t rtt,
                                uint32_t frac_lost, uint32_t jitter,
                                uint32_t lost_count) = 0;
  virtual void OnHandleNack(const std::vector<uint16_t> &seqs,
                            uint32_t local_ssrc) = 0;
  virtual void OnRembBitrate(uint32_t local_ssrc, uint32_t bitrate) = 0;
  virtual void OnRequestLocalIFrame(uint32_t local_ssrc) = 0;
  virtual void OnHandleXr(uint32_t local_ssrc, uint32_t lrr, uint32_t dlrr) = 0;
  virtual void OnHandleTwcc(const TwccPacket &twcc_packet) = 0;
  virtual void OnHandleSenderReport(uint32_t remote_ssrc, uint32_t ntp_secs,
                                    uint32_t ntp_frac,
                                    uint32_t rtp_timestamp) = 0;
  virtual void OnHandleBye() = 0;
};
*/

class RtcpHandler {
 public:
  explicit RtcpHandler(PeerConnection& pc);

  std::string ToString() const;

  int HandleRtcpPacket(const std::vector<char>& vBufReceive);
  // uint32_t CreateRtcpPacket(RTCPData::shared data, std::vector<char>&
  // o_rtcpPacketBin);

  // uint64_t GetTimeStamps(uint32_t local_ssrc, uint32_t remote_ssrc, uint32_t
  // rtp_timestamp);

  int HandleSenderReport();
  int HandleReceiverReport();
  int HandleRtpFeedbackReport();
  int HandlePayloadFeedbackReport();
  int HandleXrExtendReport();

  uint32_t CreateSenderReport(std::vector<char>& o_rtcpPacketBin);
  uint32_t CreateReceiverReport(std::vector<char>& o_rtcpPacketBin);
  uint32_t CreatePLIReport(std::vector<char>& o_rtcpPacketBin);
  // uint32_t CreateNackReport(std::vector<char>& o_rtcpPacketBin);
  uint32_t CreateXr(std::vector<char>& o_rtcpPacketBin);
  // uint32_t CreateBye(RtcpByeInfo& bye, std::vector<char>& o_rtcpPacketBin);
 private:
  // std::map<uint32_t, std::deque<SenderReportInfo>> remote_send_report_map_;
  uint64_t last_receive_rrtr_time_ms_ = 0;
  uint64_t last_rrtr_ntp_time_ = 0;
  uint32_t rrtr_ssrc_ = 0;

#ifndef __USE_SINGLE_THREAD__
  std::recursive_mutex mutex_;
#endif

  PeerConnection& belongingPeerConnection_;
};

#endif  // RTP_RTCP_RTCP_HANDLER_H_
