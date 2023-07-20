#ifndef RTP_SRTP_SRTP_HANDLER_H_
#define RTP_SRTP_SRTP_HANDLER_H_

#include <string>
#include <vector>

#include "srtp.h"

class PeerConnection;

class SrtpHandler {
 public:
  explicit SrtpHandler(PeerConnection &pc);
  ~SrtpHandler();  // no need virtual

  int UnprotectRtp(std::vector<char> *io_vBufForSrtp);
  int UnprotectRtcp(std::vector<char> *io_vBufForSrtp);
  int ProtectRtp(std::vector<char> *io_vBufForSrtp);
  int ProtectRtcp(std::vector<char> *io_vBufForSrtp);

  /**
   * Sets a key pair for the RTP and RTCP channel
   * @param sendingKey The key for protecting data
   * @param receivingKey The key for unprotecting data
   * @return true if everything is ok
   */
  bool SetRtpParams(const std::string &sending_key,
                    const std::string &receiving_key);

  // void SetStreamId(const std::string &stream_id);

 private:
  enum TransmissionType { kSending, kReceiving };

  bool ConfigureSrtpSession(srtp_t *session, const std::string &key,
                            TransmissionType type);

  std::string TransmissionTypeToString(TransmissionType type) {
    switch (type) {
      case kSending:
        return "kSending";
      case kReceiving:
        return "kReceiving";
      default:
        return "UnknownValue[" + std::to_string(static_cast<int>(type)) + "]";
    }
  }

 public:
  PeerConnection &belongingPeerConnection_;

 private:
  static bool isInitSrtp_;

  // key可能要保存，避免重启服务丢失
  std::string sending_key_;    // when send rtp
  std::string receiving_key_;  // when recv srtp
  srtp_t send_session_ = nullptr;
  srtp_t receive_session_ = nullptr;
  bool active_ = false;
};

#endif  // RTP_SRTP_SRTP_HANDLER_H_
