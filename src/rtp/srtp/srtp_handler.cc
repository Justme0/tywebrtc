// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// ref
// https://github.com/lynckia/licode/blob/master/erizo/src/erizo/SrtpChannel.cpp

#include "src/rtp/srtp/srtp_handler.h"

#include <cassert>
#include <cstring>
#include <mutex>

#include "tylib/codec/codec.h"

#include "src/log/log.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

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
  int ret = 0;

  if (send_session_ != nullptr) {
    ret = srtp_dealloc(send_session_);
    if (ret) {
      tylog("srtp_dealloc send_session fail, ret=%d", ret);
    }
    send_session_ = nullptr;
  }

  if (receive_session_ != nullptr) {
    ret = srtp_dealloc(receive_session_);
    if (ret) {
      tylog("srtp_dealloc receive_session fail, ret=%d", ret);
    }
    receive_session_ = nullptr;
  }
}

// @return 是否成功
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
  policy.next = nullptr;
  std::string decodedKey = tylib::Base64Decode(key);
  tylog(
      "transmit type=%s. input key len=%zu, data=%s, after base64 decode, "
      "len=%zu.",
      TransmissionTypeToString(type).data(), key.size(), key.data(),
      decodedKey.size());

  // decodedKey.push_back('\0');                 // NOTE?

  // srtp_create temp use policy? NOTE key is temp
  policy.key = reinterpret_cast<unsigned char *>(
      &decodedKey[0]);  // C++17 can use data()
  srtp_err_status_t ret = srtp_create(session, &policy);
  if (0 != ret) {
    tylog("srtp_create fail ret=%d, key=%s, transmit type=%s", ret, key.data(),
          TransmissionTypeToString(type).data());

    return false;
  }

  return true;
}

// @brief 设置RTP加解密key，构造session
// @return 是否成功
bool SrtpHandler::SetRtpParams(const std::string &sending_key,
                               const std::string &receiving_key) {
  bool ok = false;
  active_ = false;

  ok = ConfigureSrtpSession(&send_session_, sending_key, kSending);
  if (!ok) {
    tylog("configureSrtpSession send key fail");

    return false;
  }
  assert(nullptr != send_session_);

  ok = ConfigureSrtpSession(&receive_session_, receiving_key, kReceiving);
  if (!ok) {
    tylog("configureSrtpSession recv key fail");

    return false;
  }
  assert(nullptr != receive_session_);

  sending_key_ = sending_key;  // OPT: use move
  receiving_key_ = receiving_key;

  active_ = true;
  return true;
}

// Encrypted portion is only payload.
// https://mp.weixin.qq.com/s/u7eStMiCGxNyEWsYkG8UYg
int SrtpHandler::ProtectRtp(std::vector<char> *io_vBufForSrtp) {
  if (this->belongingPeerConnection_.bNotUseSrtp) {
    return 0;
  }

  if (!active_) {
    tylog("error active=%d", active_);
    return -1;
  }

  size_t len = io_vBufForSrtp->size();
  const size_t kOriginLen = len;

  assert(nullptr != send_session_);

  // protect may increase SRTP_MAX_TRAILER_LEN
  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/srtp/srtp.c;l=2352
  io_vBufForSrtp->resize(io_vBufForSrtp->size() + SRTP_MAX_TRAILER_LEN);
  int ret = srtp_protect(send_session_,
                         reinterpret_cast<uint8_t *>(io_vBufForSrtp->data()),
                         &len, 0);
  if (ret) {
    tylog("srtp_protect ret=%d", ret);
    return ret;
  }

  tylog("origin len=%zu, after protect len=%zu, before srtp we prepare to %zu",
        kOriginLen, len, io_vBufForSrtp->size());
  assert(kOriginLen <= len && len <= io_vBufForSrtp->size());
  io_vBufForSrtp->resize(len);

  return 0;
}

int SrtpHandler::UnprotectRtp(std::vector<char> *io_vBufForSrtp) {
  if (this->belongingPeerConnection_.bNotUseSrtp) {
    return 0;
  }

  if (!active_) {
    tylog("error active=%d", active_);
    return -1;
  }

  assert(nullptr != receive_session_);

  size_t len = io_vBufForSrtp->size();
  srtp_err_status_t ret =
      srtp_unprotect(receive_session_,
                     reinterpret_cast<uint8_t *>(io_vBufForSrtp->data()), &len);
  if (ret) {
    // if return srtp_err_status_replay_old=10
    // https://mp.weixin.qq.com/s/u7eStMiCGxNyEWsYkG8UYg
    //
    // if return srtp_err_status_replay_fail=9,
    // maybe unordered, nack retransmit, not error
    tylog("warning: srtp unprotect ret=%d", ret);

    return ret;
  }

  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/srtp/srtp.c;l=2718
  tylog("after unprotect, len=%zu, origin=%zu", len, io_vBufForSrtp->size());
  assert(len <= io_vBufForSrtp->size());

  io_vBufForSrtp->resize(len);

  return 0;
}

// Encrypt portion after header, so sender SSRC isn't encrypted.
// WebRTC 传输安全机制第二话：深入显出 SRTP 协议
// https://mp.weixin.qq.com/s/u7eStMiCGxNyEWsYkG8UYg
int SrtpHandler::ProtectRtcp(std::vector<char> *io_vBufForSrtp) {
  if (this->belongingPeerConnection_.bNotUseSrtp) {
    return 0;
  }

  if (!active_) {
    tylog("error active=%d", active_);
    return -1;
  }

  size_t len = io_vBufForSrtp->size();
  const size_t kOriginLen = len;

  assert(nullptr != send_session_);

  // protect may increase SRTP_MAX_TRAILER_LEN + 4
  // NOTE add 4, refer `srtp_protect_rtcp` interface doc
  // 4 means sizeof(srtcp_trailer_t)
  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/include/srtp_priv.h;l=237
  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/srtp/srtp.c;l=4078
  io_vBufForSrtp->resize(io_vBufForSrtp->size() + SRTP_MAX_TRAILER_LEN + 4);
  int ret = srtp_protect_rtcp(
      send_session_, reinterpret_cast<uint8_t *>(io_vBufForSrtp->data()), &len,
      0);
  if (ret) {
    tylog("srtp_protect_rtcp ret=%d", ret);
    return ret;
  }

  tylog("origin len=%zu, after protect len=%zu, before srtp we prepare to %zu",
        kOriginLen, len, io_vBufForSrtp->size());
  assert(kOriginLen <= len && len <= io_vBufForSrtp->size());
  io_vBufForSrtp->resize(len);

  return 0;
}

// taylor may error
int SrtpHandler::UnprotectRtcp(std::vector<char> *io_vBufForSrtp) {
  if (this->belongingPeerConnection_.bNotUseSrtp) {
    return 0;
  }

  if (!active_) {
    tylog("error active=%d", active_);
    return -1;
  }

  assert(nullptr != receive_session_);

  size_t len = io_vBufForSrtp->size();
  srtp_err_status_t ret = srtp_unprotect_rtcp(
      receive_session_, reinterpret_cast<uint8_t *>(io_vBufForSrtp->data()),
      &len);
  if (ret) {
    tylog("error srtp unprotect ret=%d", ret);

    return ret;
  }

  // https://source.chromium.org/chromium/chromium/src/+/main:third_party/libsrtp/srtp/srtp.c;l=4346
  tylog("after unprotect, len=%zu, origin=%zu", len, io_vBufForSrtp->size());
  assert(len <= io_vBufForSrtp->size());

  io_vBufForSrtp->resize(len);

  return 0;
}

}  // namespace tywebrtc
