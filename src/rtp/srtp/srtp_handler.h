// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_SRTP_SRTP_HANDLER_H_
#define SRC_RTP_SRTP_SRTP_HANDLER_H_

#include <string>
#include <vector>

#include "srtp2/srtp.h"

namespace tywebrtc {

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
  PeerConnection &belongingPC_;

 private:
  static bool isInitSrtp_;

  // key可能要保存，避免重启服务丢失
  std::string sending_key_;    // when send rtp
  std::string receiving_key_;  // when recv srtp
  srtp_t send_session_ = nullptr;
  srtp_t receive_session_ = nullptr;
  bool active_ = false;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_SRTP_SRTP_HANDLER_H_