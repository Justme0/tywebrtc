// ref
// https://github.com/lynckia/licode/blob/master/erizo/src/erizo/SrtpChannel.cpp

#include "rtp/srtp/srtp_handler.h"

#include <cassert>
#include <cstring>
#include <mutex>

#include "tylib/codec/codec.h"

#include "log/log.h"

bool SrtpHandler::isInitSrtp_ = false;

SrtpHandler::SrtpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {
  if (SrtpHandler::isInitSrtp_) {
    return;
  }

  // OPT: use call_once cause coredump, why...
  srtp_err_status_t ret = srtp_init();
  if (ret) {
    tylog("NOTE!!! in constructor srtp_init fail, ret=%d, cannot return err",
          ret);
  }

  SrtpHandler::isInitSrtp_ = true;
}

SrtpHandler::~SrtpHandler() {
  if (send_session_ != NULL) {
    srtp_dealloc(send_session_);
    send_session_ = NULL;
  }

  if (receive_session_ != NULL) {
    srtp_dealloc(receive_session_);
    receive_session_ = NULL;
  }
}

bool SrtpHandler::ConfigureSrtpSession(srtp_t *session, const std::string &key,
                                       enum TransmissionType type) {
  srtp_policy_t policy;
  memset(&policy, 0, sizeof(policy));
  srtp_crypto_policy_set_aes_cm_128_hmac_sha1_80(&policy.rtp);
  srtp_crypto_policy_set_aes_cm_128_hmac_sha1_80(&policy.rtcp);

  if (type == kSending) {
    policy.ssrc.type = ssrc_any_outbound;
  } else {
    policy.ssrc.type = ssrc_any_inbound;
  }

  policy.ssrc.value = 0;
  policy.window_size = 8192;
  policy.allow_repeat_tx = 1;
  policy.next = NULL;
  std::string decodedKey = tylib::Base64Decode(key);
  tylog("decode key=%s", decodedKey.data());
  decodedKey.push_back('\0');  // NOTE?

  policy.key = reinterpret_cast<unsigned char *>(
      &decodedKey[0]);  // C++17 can use data()
                        // srtp_create temp use policy? NOTE key is temp
  srtp_err_status_t ret = srtp_create(session, &policy);
  if (0 != ret) {
    tylog("srtp_create ret=%d", ret) return false;
  }

  return true;
}

bool SrtpHandler::SetRtpParams(const std::string &sending_key,
                               const std::string &receiving_key) {
  if (ConfigureSrtpSession(&send_session_, sending_key, kSending) &&
      ConfigureSrtpSession(&receive_session_, receiving_key, kReceiving)) {
    sending_key_.assign(sending_key);
    receiving_key_.assign(receiving_key);

    active_ = true;
    return true;
  } else {
    active_ = false;
    return false;
  }
}

int SrtpHandler::ProtectRtp(std::vector<char> *io_vBufForSrtp) {
  if (!active_) {
    tylog("error active=%d", active_);
    return -1;
  }

  int len = io_vBufForSrtp->size();
  const int kOriginLen = len;

  // protect may increase SRTP_MAX_TRAILER_LEN
  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/srtp/srtp.c;l=2352
  io_vBufForSrtp->resize(io_vBufForSrtp->size() + SRTP_MAX_TRAILER_LEN);
  int ret = srtp_protect(send_session_, io_vBufForSrtp->data(), &len);
  if (ret) {
    tylog("srtp_protect ret=%d", ret);
    return ret;
  }

  tylog("origin len=%d, after protect len=%d, before srtp we prepare to %zu",
        kOriginLen, len, io_vBufForSrtp->size());
  assert(kOriginLen <= len && len <= static_cast<int>(io_vBufForSrtp->size()));
  io_vBufForSrtp->resize(len);

  return 0;
}

int SrtpHandler::UnprotectRtp(std::vector<char> *io_vBufForSrtp) {
  if (!active_) {
    tylog("error active=%d", active_);
    return -1;
  }

  int len = io_vBufForSrtp->size();
  srtp_err_status_t ret =
      srtp_unprotect(receive_session_, io_vBufForSrtp->data(), &len);
  if (ret) {
    tylog("error srtp_unprotect ret=%d", ret);
    return ret;
  }

  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/srtp/srtp.c;l=2718
  tylog("after unprotect, len=%d, origin=%zu", len, io_vBufForSrtp->size());
  assert(len <= static_cast<int>(io_vBufForSrtp->size()));

  io_vBufForSrtp->resize(len);

  return 0;
}

int SrtpHandler::ProtectRtcp(std::vector<char> *io_vBufForSrtp) {
  if (!active_) {
    tylog("error active=%d", active_);
    return -1;
  }

  int len = io_vBufForSrtp->size();
  const int kOriginLen = len;

  // protect may increase SRTP_MAX_TRAILER_LEN + 4
  // NOTE add 4, refer `srtp_protect_rtcp` interface doc
  // 4 means sizeof(srtcp_trailer_t)
  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/include/srtp_priv.h;l=237
  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/srtp/srtp.c;l=4078
  io_vBufForSrtp->resize(io_vBufForSrtp->size() + SRTP_MAX_TRAILER_LEN + 4);
  int ret = srtp_protect_rtcp(send_session_, io_vBufForSrtp->data(), &len);
  if (ret) {
    tylog("srtp_protect_rtcp ret=%d", ret);
    return ret;
  }

  tylog("origin len=%d, after protect len=%d, before srtp we prepare to %zu",
        kOriginLen, len, io_vBufForSrtp->size());
  assert(kOriginLen <= len && len <= static_cast<int>(io_vBufForSrtp->size()));
  io_vBufForSrtp->resize(len);

  return 0;
}

int SrtpHandler::UnprotectRtcp(std::vector<char> *io_vBufForSrtp) {
  if (!active_) {
    tylog("error active=%d", active_);
    return -1;
  }

  int len = io_vBufForSrtp->size();
  srtp_err_status_t ret =
      srtp_unprotect_rtcp(receive_session_, io_vBufForSrtp->data(), &len);
  if (ret) {
    tylog("error srtp_unprotect ret=%d", ret);
    return ret;
  }

  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/srtp/srtp.c;l=4346
  tylog("after unprotect, len=%d, origin=%zu", len, io_vBufForSrtp->size());
  assert(len <= static_cast<int>(io_vBufForSrtp->size()));

  io_vBufForSrtp->resize(len);

  return 0;
}
