// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// DTLS state
// 0  HelloRequest
// 1  ClientHello
// 2  ServerHello
// 4  NewSessionTicket
// 11 Certificate
// 12 ServerKeyExchange
// 13 CertificateRequest
// 14 ServerHelloDone
// 15 CertificateVerify
// 16 ClientKeyExchange
// 20 Finished

#include "src/dtls/dtls_handler.h"

#include <cassert>
#include <cinttypes>

#include "openssl/bio.h"
#include "tylib/codec/codec.h"
#include "tylib/ip/ip.h"
#include "tylib/string/format_string.h"
#include "tylib/time/time_util.h"
#include "tylib/time/timer.h"

#include "src/dtls/certificate_key.h"
#include "src/log/log.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

const int SRTP_MASTER_KEY_KEY_LEN = 16;
const int SRTP_MASTER_KEY_SALT_LEN = 14;
const int DTLS_MTU = 1100;

const char* DtlsHandler::DefaultSrtpProfile = "SRTP_AES128_CM_SHA1_80";

//证书格式，包含公钥、加密算法、有效期等参数。
X509* DtlsHandler::mCert = nullptr;

// key的EVP，可封装rsa、dsa、ecc等key
EVP_PKEY* DtlsHandler::privkey = nullptr;

int DummyCb(int /*preverify_ok*/, X509_STORE_CTX* /*x509_ctx*/) { return 1; }

// static bool DropUpPkgRand() {
//   int lossRate = 0;
//   std::ifstream f("testLost.txt");
//   if (f) {
//     f >> lossRate;  // up loss rate
//   }
//
//   int dopKey = rand() % 100;
//
//   if (dopKey < lossRate) {
//     tylog("up lostrate=%d%%", lossRate);
//     return true;
//   }
//
//   return false;
// }

// static bool DropDownPkgRand() {
//   int lossRate = 0;
//   std::ifstream f("testLost.txt");
//   if (f) {
//     int upLossRate;
//     f >> upLossRate;
//     f >> lossRate;  // down loss rate
//   }
//
//   int dopKey = rand() % 100;
//
//   if (dopKey < lossRate) {
//     tylog("down lostrate=%d%%", lossRate);
//     return true;
//   }
//
//   return false;
// }

static inline const char* GetSslBuffStateString(int errCode) {
  switch (errCode) {
    case SSL_ERROR_NONE:
      return "SSL_ERROR_NONE";
    case SSL_ERROR_SSL:
      return "SSL_ERROR_SSL";
    case SSL_ERROR_WANT_READ:
      return "SSL_ERROR_WANT_READ";
    case SSL_ERROR_WANT_WRITE:
      return "SSL_ERROR_WANT_WRITE";
    case SSL_ERROR_WANT_X509_LOOKUP:
      return "SSL_ERROR_WANT_X509_LOOKUP";
    case SSL_ERROR_SYSCALL:
      // look at error stack/return value/errno
      return "SSL_ERROR_SYSCALL";
    case SSL_ERROR_ZERO_RETURN:
      return "SSL_ERROR_ZERO_RETURN";
    case SSL_ERROR_WANT_CONNECT:
      return "SSL_ERROR_WANT_CONNECT";
    case SSL_ERROR_WANT_ACCEPT:
      return "SSL_ERROR_WANT_ACCEPT";
    default:
      return "Unknown SSL ERROR";
  }
}

static inline std::string GetSslStateString(int stateCode) {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
  SSL ssl;
  ssl.state = stateCode;
  return SSL_state_string_long(&ssl);
#else
  return "shit_cannot_get_description_in_new_openssl_version:(";
  (void)stateCode;
#endif
}

// static inline int dummy_cb(int d, X509_STORE_CTX* x) { return 1; }

void SSLInfoCallback(const SSL* s, int where, int callbackRet) {
  int ret = 0;

  if (where & SSL_CB_HANDSHAKE_DONE) {
    ret = SSL_get_verify_result(s);  // 0 is X509_V_OK, means ok
    if (ret) {
      tylog("SSL_get_verify_result fail, ret=%d", ret);
      // not return ?
    } else {
      tylog("SSL_CB_HANDSHAKE_DONE ok, callbackRet=%d, ssl state=%d[%s]",
            callbackRet, SSL_get_state(s), SSL_state_string_long(s));
    }
  }

  const char* strState = "undefined";
  int w = where & ~SSL_ST_MASK;

  if (w & SSL_ST_CONNECT) {
    strState = "SSL_connect";
  } else if (w & SSL_ST_ACCEPT) {
    strState = "SSL_accept";
  }

  const char* str = "undefined";
  if (where & SSL_CB_LOOP) {
    tylog("where=%#x has SSL_CB_LOOP flag, %s", where,
          SSL_state_string_long(s));
  } else if (where & SSL_CB_ALERT) {
    str = (where & SSL_CB_READ) ? "read" : "write";
    tylog(
        "SSL3[State %s] where=%#x, callbackRet=%d, r/w=%s, alert type=%s, "
        "alert "
        "desc=%s",
        strState, where, callbackRet, str,
        SSL_alert_type_string_long(callbackRet),
        SSL_alert_desc_string_long(callbackRet));
  } else if (where & SSL_CB_EXIT) {
    if (callbackRet == 0) {
      tylog("succ[%d] in %s", callbackRet, SSL_state_string_long(s));
    } else if (callbackRet < 0) {
      tylog("error[%d] in %s", callbackRet, SSL_state_string_long(s));
    } else if (callbackRet > 0) {
      tylog("callbackRet=%d, %s", callbackRet, SSL_state_string_long(s));
    }
  }
}

int SSLVerifyCallback(int ok, X509_STORE_CTX* store) {
  if (!ok) {
    char data[256], data2[256];
    X509* cert = X509_STORE_CTX_get_current_cert(store);
    X509_NAME_oneline(X509_get_issuer_name(cert), data, sizeof(data));
    X509_NAME_oneline(X509_get_subject_name(cert), data2, sizeof(data2));
  }

  // In peer-to-peer mode, no root cert / certificate authority was
  // specified, so the libraries knows of no certificate to accept,
  // and therefore it will necessarily call here on the first cert it
  // tries to verify.
  if (!ok) {
    int err = X509_STORE_CTX_get_error(store);

    tylog("ssl verify error: %d", err);

    // peer-to-peer mode: allow the certificate to be self-signed,
    // assuming it matches the digest that was specified.
    if (err == X509_V_ERR_DEPTH_ZERO_SELF_SIGNED_CERT) {
      tylog("Accepted self-signed peer certificate authority");
      ok = 1;
    }
  }
  if (!ok) {
    tylog("Ignoring cert error while verifying cert chain");
    ok = 1;
  }

  return ok;
}

long DtlsOutBIOCallback(BIO* bio, int cmd, const char* argp, int argi,
                        long /*argl*/, long ret) {
  long r = 1;

  if (BIO_CB_RETURN & cmd) {
    r = ret;
  }

  // convert BIO to string to print
  // tylog("cmd=%d, bio num=%d, argi=%d, method=%s", cmd, bio->num, argi,
  // bio->method->name);

  if (BIO_CB_WRITE == cmd) {
    DtlsHandler* d = reinterpret_cast<DtlsHandler*>(BIO_get_callback_arg(bio));
    if (argp && 0 < argi && d) {
      int errCode = d->WriteDtlsPacket(argp, argi);
      if (errCode) {
        tylog("writeDtlsPacket ret=%d", errCode);
        // return ret ?
      }
    }
  }

  return r;
}

void DtlsHandler::InitOpensslAndCert() {
#if (OPENSSL_VERSION_NUMBER > 0x10100000L)
  OPENSSL_init_ssl(0, nullptr);
  OPENSSL_init_crypto(OPENSSL_INIT_ADD_ALL_CIPHERS, nullptr);
  OPENSSL_init_crypto(OPENSSL_INIT_ADD_ALL_DIGESTS, nullptr);
#else
  SSL_library_init();
  SSL_load_error_strings();
  ERR_load_crypto_strings();
#endif
  int ret = GetCertificateAndKey(DtlsHandler::mCert, DtlsHandler::privkey);
  if (ret) {
    tylog("getCertificateAndKey error, ret=%d", ret);
  }
}

DtlsHandler::DtlsHandler(PeerConnection& pc, bool isServer)
    : belongingPC_(pc),
      m_isServer(isServer),
      mHandshakeCompleted(false),
      mGetKeyFlag(false),
      m_SendBuffNum(0),
      m_CheckTime(g_now_ms),
      m_SSl_BuffState(SSL_ERROR_NONE),
      m_ResetFlag(false),
      m_ReSendTime(0),
      mHandshakeFail(false),
      m_ClientKeySendTime(0),
      m_StmDirect(kSendRecv),
      m_IsHandshakeCanComplete(false),
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
      m_LastSslState(SSL_ST_BEFORE),
#else
      m_LastSslState(TLS_ST_BEFORE),
#endif
      m_startFlag(false),
      dtlsTimer_(*this) {
  InitOpensslAndCert();

  memset(m_SendBuff, 0, sizeof(m_SendBuff));

  tylog("Creating Dtls factory, Openssl v %s", OPENSSL_VERSION_TEXT);

  mContext = SSL_CTX_new(DTLS_method());
  assert(mContext);
  int r = SSL_CTX_use_certificate(mContext, mCert);
  assert(r == 1);
  r = SSL_CTX_use_PrivateKey(mContext, privkey);
  assert(r == 1);
  SSL_CTX_set_cipher_list(mContext, "ALL:!ADH:!LOW:!EXP:!MD5:@STRENGTH");
  SSL_CTX_set_info_callback(mContext, SSLInfoCallback);
  SSL_CTX_set_verify(mContext,
                     SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT,
                     SSLVerifyCallback);
  SSL_CTX_set_options(mContext, SSL_OP_NO_QUERY_MTU);
  // Enable ECDH ciphers.
  SSL_CTX_set_ecdh_auto(mContext, 1);

  // Set SRTP profiles
  r = SSL_CTX_set_tlsext_use_srtp(mContext, DefaultSrtpProfile);
  assert(r == 0);
  SSL_CTX_set_verify_depth(mContext, 2);
  SSL_CTX_set_read_ahead(mContext, 1);

  mSsl = SSL_new(mContext);
  assert(mSsl != nullptr);
  // DTLS & MTU
  // https://segmentfault.com/a/1190000040041788
  SSL_set_mtu(mSsl, DTLS_MTU);
  mInBio = BIO_new(BIO_s_mem());

  mOutBio = BIO_new(BIO_s_mem());
  BIO_ctrl(mOutBio, BIO_CTRL_DGRAM_SET_MTU, 0, nullptr);
  BIO_set_callback(mOutBio, DtlsOutBIOCallback);
  BIO_set_callback_arg(mOutBio, reinterpret_cast<char*>(this));

  SSL_set_bio(mSsl, mInBio, mOutBio);
}

DtlsHandler::~DtlsHandler() {
  tylog("in destructor closing dtls, %s", ToString().data());

  if (mSsl != nullptr) {
    // SSL_shutdown(mSsl);
    SSL_free(mSsl);
    mSsl = nullptr;
  }

  SSL_CTX_free(mContext);
}

// @brief 只在HandshakeCompleted()被调用
int DtlsHandler::OnHandshakeCompleted_() {
  tylog("isServer: %d clientKey: %s, serverKey: %s, swap keys if server",
        m_isServer, sendingRtpKey_.c_str(), receivingRtpKey_.c_str());

  if (m_isServer) {
    // If we are server, we swap the keys
    tylog("message: swapping keys, isServer: %d", m_isServer);
    sendingRtpKey_.swap(receivingRtpKey_);
  }

  bool ok =
      belongingPC_.srtpHandler_.SetRtpParams(sendingRtpKey_, receivingRtpKey_);
  if (!ok) {
    tylog("NOTE!!! srtpHandler_ setRtpParams fail");

    return -1;
  }

  SET_PC_STATE(belongingPC_, EnumStateMachine::DTLS_DONE);

  return 0;
}

// 目前实参都是true，再次确认why？
// TODO 调用处判断返回值
int DtlsHandler::HandshakeCompleted(bool bSessionCompleted) {
  int ret = 0;

  if (mGetKeyFlag) {
    tylog("We get Key, no need to do it, bSessionCompleted=%d, %s",
          bSessionCompleted, ToString().data());
    mHandshakeCompleted = bSessionCompleted;

    return 0;
  }

  char fprint[MAX_FP_SIZE]{};
  if (!GetRemoteFingerprint(fprint)) {
    tylog("getRemoteFingerprint err, peer not authenticate, return succ %s.",
          ToString().data());

    return 0;
  }

  bool checkOk = CheckFingerprint_(fprint, strlen(fprint));
  if (!checkOk) {
    // 两次获取对端指纹，应当相同
    tylog("check fingerprint fail %s", ToString().data());

    return -2;
  }

  // const int SRTP_MASTER_KEY_LEN = 30;
  unsigned char material[SRTP_MASTER_KEY_LEN << 1];

  // SSL_export_keying_material() returns 0 or -1 on failure or 1 on success.
  int isSucc =
      SSL_export_keying_material(mSsl, material, sizeof(material),
                                 "EXTRACTOR-dtls_srtp", 19, nullptr, 0, 0);
  if (1 != isSucc) {
    tylog("SSL_export_keying_material err, isSucc=%d %s", isSucc,
          ToString().data());

    return -3;
  }

  int offset = 0;

  char cKey[SRTP_MASTER_KEY_KEY_LEN + SRTP_MASTER_KEY_SALT_LEN];
  char sKey[SRTP_MASTER_KEY_KEY_LEN + SRTP_MASTER_KEY_SALT_LEN];

  memcpy(cKey, &material[offset], SRTP_MASTER_KEY_KEY_LEN);
  offset += SRTP_MASTER_KEY_KEY_LEN;

  memcpy(sKey, &material[offset], SRTP_MASTER_KEY_KEY_LEN);
  offset += SRTP_MASTER_KEY_KEY_LEN;

  memcpy(cKey + SRTP_MASTER_KEY_KEY_LEN, &material[offset],
         SRTP_MASTER_KEY_SALT_LEN);
  offset += SRTP_MASTER_KEY_SALT_LEN;

  memcpy(sKey + SRTP_MASTER_KEY_KEY_LEN, &material[offset],
         SRTP_MASTER_KEY_SALT_LEN);
  offset += SRTP_MASTER_KEY_SALT_LEN;

  // check len with MAX_SRTP_KEY_LEN
  // OPT escape copy string
  std::string rawClientKey(cKey,
                           SRTP_MASTER_KEY_KEY_LEN + SRTP_MASTER_KEY_SALT_LEN);
  sendingRtpKey_ = tylib::Base64Encode(rawClientKey);
  std::string rawClientKeyRecover = tylib::Base64Decode(sendingRtpKey_);
  tylog("rawClientKey=%s, sendingRtpKey_=%s, rawClientKeyRecover=%s",
        rawClientKey.data(), sendingRtpKey_.data(), rawClientKeyRecover.data());

  assert(rawClientKey == rawClientKeyRecover);

  receivingRtpKey_ = tylib::Base64Encode(
      std::string(sKey, SRTP_MASTER_KEY_KEY_LEN + SRTP_MASTER_KEY_SALT_LEN));

  tylog("ClientKey: %s", sendingRtpKey_.c_str());
  tylog("ServerKey: %s", receivingRtpKey_.c_str());

  SRTP_PROTECTION_PROFILE* srtp_profile = SSL_get_selected_srtp_profile(mSsl);
  if (srtp_profile) {
    tylog("SRTP Extension negotiated profile=%s", srtp_profile->name);
  }

  mGetKeyFlag = true;  // 本变量只在此处修改

  if (bSessionCompleted) {
    mHandshakeCompleted = true;
  }

  tylog("bSessionCompleted=%d %s", bSessionCompleted, ToString().data());

  ret = OnHandshakeCompleted_();
  if (ret) {
    tylog("onHandshakeCompleted fail, ret=%d", ret);

    return ret;
  }

  return 0;
}

// 只在处理DTLS包结尾处调用
void DtlsHandler::CheckHandshakeComplete_() {
  if (mHandshakeCompleted) {
    tylog("handshake completed, no need check %s", ToString().data());

    return;
  }

  int r = SSL_do_handshake(mSsl);
  m_SSl_BuffState = SSL_get_error(mSsl, r);
  tylog("handshake not complete, after SSL_do_handshake(): %s",
        ToString().data());

  if (m_SSl_BuffState == SSL_ERROR_NONE) {
    tylog("Do handshakeCompleted ssl_buff ok");
    const bool kSessionCompleted = true;
    int ret = HandshakeCompleted(kSessionCompleted);
    if (ret) {
      tylog("handshakeCompleted fail, ret=%d", ret);
    }
  }

  tylog("as client, %s", ToString().data());

  if (m_isServer) {
    return;
  }

  // as DTLS client:

  if (!m_IsHandshakeCanComplete) {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
    m_IsHandshakeCanComplete =
        (SSL3_ST_CR_SESSION_TICKET_A == SSL_get_state(mSsl) ||
         SSL3_ST_CR_FINISHED_A == SSL_get_state(mSsl));
#else
    m_IsHandshakeCanComplete = (TLS_ST_CW_FINISHED == SSL_get_state(mSsl));
#endif
  }

  if (!m_IsHandshakeCanComplete) {
    return;
  }

  // TODO
  tylog("Do handshakeCompleted CW_FINISHED");
  // const bool kSessionCompleted = true;
  // int ret = HandshakeCompleted(kSessionCompleted); // or false ?
  // if (ret) {
  //   tylog("handshakeCompleted fail, ret=%d", ret);
  // }
}

int DtlsHandler::DoDataChannel_(const std::vector<char>& vBufReceive) {
  int ret = 0;

  ret = this->belongingPC_.dataChannelHandler_.InitSocket();
  if (ret) {
    tylog("init sctp socket ret=%d.", ret);
    assert(!"init sctp socket fail");  // tmp

    return ret;
  }

  BIO_reset(mInBio);
  BIO_reset(mOutBio);
  BIO_write(mInBio, vBufReceive.data(), vBufReceive.size());
  while (BIO_ctrl_pending(mInBio) > 0) {
    char sctp_read_buf[8092];
    int size = SSL_read(mSsl, sctp_read_buf, sizeof(sctp_read_buf));
    if (size <= 0) {
      m_SSl_BuffState = SSL_get_error(mSsl, size);
      tylog("SSL_read ret=%d, %s", ret, ToString().data());

      break;
    }

    this->belongingPC_.dataChannelHandler_.HandleDataChannelPacket(
        sctp_read_buf, size);
  }

  return 0;
}

// @brief 收到DTLS包处理
// @param [in] vBufReceive DTLS包
// @return 处理成功返0
int DtlsHandler::HandleDtlsPacket(const std::vector<char>& vBufReceive) {
  belongingPC_.sdpHandler_.bNotUseSrtp = false;  // hack
  DumpRecvPacket(vBufReceive);

  int ret = 0;

  tylog("read Dtls message len:%zu %s", vBufReceive.size(), ToString().data());
  // OPT: parse dtls e.g. alert pkt

  // do data channel
  if (this->mHandshakeCompleted) {
    tylog("data channel recv data size=%zu.", vBufReceive.size());

    return DoDataChannel_(vBufReceive);
  }

  if (!m_startFlag && m_isServer) {
    tylog("as DTLS server, recv DTLS packet before user-candidate STUN");

    ret = StartDTLS();
    if (ret) {
      tylog("start dtls fail, ret=%d", ret);

      return ret;
    }
  }

  // todo dump
  //    WebRtcDumpVideoPkg((char*)data, len,  WEB_RTC_DUMP_ICE_IN);

  int curSslState = SSL_get_state(mSsl);

  if (m_LastSslState != curSslState) {
    // 上报监控，初始一定不同无需上报
    int kInitLastSslState = 0;
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
    kInitLastSslState = SSL_ST_BEFORE;
#else
    kInitLastSslState = TLS_ST_BEFORE;
#endif
    if (m_LastSslState != kInitLastSslState) {
      tylog("last ssl is not same as current: %s, now set m_ResetFlag to true",
            ToString().data());
    }

    m_ResetFlag = true;
  } else {
    tylog("ssl state is same as last=%d[%s]", curSslState,
          SSL_state_string_long(mSsl));
  }

  m_LastSslState = curSslState;

  // 我方已完成握手，但仍然收到DTLS包则重发之前备份的包
  const int kMaxReSendTime = 5;
  if (mHandshakeCompleted && (kMaxReSendTime >= m_ReSendTime)) {
    ++m_ReSendTime;

    if (0 == m_SendBuffNum) {
      tylog("why: handshake completed, last sent package not exist, %s",
            ToString().data());
    }
    assert(0 != m_SendBuffNum);

    for (int i = 0; i < m_SendBuffNum; ++i) {
      tylog("already complete, rewrite DtlsPacket i=%d, buffer len=%d", i,
            m_SendBuff[i].len);
      ret = rewriteDtlsPacket(m_SendBuff[i].buff, m_SendBuff[i].len);
      if (ret) {
        tylog("rewrite ret=%d", ret);
      }
    }

    return 0;
  }

  m_ResetFlag = true;
  int r = 0;
  r = BIO_reset(mInBio);  // todo handle return value
  r = BIO_reset(mOutBio);
  r = BIO_write(mInBio, vBufReceive.data(), vBufReceive.size());
  if (r != static_cast<int>(vBufReceive.size())) {
    tylog("error BIO write() r=%d, len=%zu, should be equal, %s", r,
          vBufReceive.size(), ToString().data());
    // error handle ?
  }
  assert(r == static_cast<int>(vBufReceive.size()));  // 是否返回
  CheckHandshakeComplete_();

  return 0;
}

// @beief 获取证书对应的指纹
// @param [in] cert 证书
// @param [out] fingerprint 指纹
//
// optimize: check buffer size, return std::string
void DtlsHandler::computeFingerprint(const X509* cert,
                                     char* fingerprint) const {
  unsigned char md[EVP_MAX_MD_SIZE];
  unsigned int n = 0;

  int r = X509_digest(cert, EVP_sha256(), md, &n);
  assert(r == 1);

  for (unsigned int i = 0; i < n; i++) {
    sprintf(fingerprint, "%02X", md[i]);
    fingerprint += 2;

    if (i < (n - 1)) {
      *fingerprint++ = ':';
    } else {
      *fingerprint++ = '\0';
    }
  }
}

// @brief 获取对端指纹
// @param fprint [out] 指纹
// @return 是否成功获取指纹
bool DtlsHandler::GetRemoteFingerprint(char* fprint) const {
  X509* x = SSL_get_peer_certificate(mSsl);
  if (nullptr == x) {
    tylog("SSL_get_peer_certificate nullptr %s", ToString().data());
    return false;
  }

  computeFingerprint(x, fprint);
  X509_free(x);

  return true;
}

// @brief 比较对端指纹和入参
// @return 是否一致
bool DtlsHandler::CheckFingerprint_(const char* fingerprint,
                                    unsigned int len) const {
  char fprint[MAX_FP_SIZE];

  if (!GetRemoteFingerprint(fprint)) {
    tylog("getRemoteFingerprint fail %s", ToString().data());
    return false;
  }

  if (0 != strncmp(fprint, fingerprint, len)) {
    tylog("compare fingerprint not same, remote=%s, inParameter=%s, %s", fprint,
          fingerprint, ToString().data());
    return false;
  }

  return true;
}

// 只在收到DTLS包开始时调用
int DtlsHandler::StartDTLS() {
  int ret = 0;

  if (m_startFlag) {
    tylog("already start DTLSRTP %s", ToString().data());

    return 0;
  }
  m_startFlag = true;

  tylog("enter startDTLS(), %s", ToString().data());

  if (m_isServer) {
    SSL_set_accept_state(mSsl);
    SSL_set_verify(mSsl, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT,
                   DummyCb);

    ret = SSL_do_handshake(mSsl);
    if (ret) {
      tylog("SSL do_handshake ret=%d.", ret);

      return ret;
    }

    m_SSl_BuffState = SSL_get_error(mSsl, ret);

  } else {
    SSL_set_connect_state(mSsl);
    tylog("dtls as client, GetKeyFlag=%d", mGetKeyFlag);

    if (mGetKeyFlag) {
      return 0;
    }

    ret = SSL_accept(mSsl);
    if (ret) {
      tylog("ssl accept fail, ret=%d, but not return", ret);
      // return ret;
    }

#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
    ret = SSL_do_handshake(mSsl);
    if (ret) {
      tylog("ssl do handshake fail, ret=%d, but not return", ret);
      // return ret;
    }
#endif

    m_SSl_BuffState = SSL_get_error(mSsl, ret);
  }

  tylog("Start DTLSRTP as %s, ret=%d, %s", (m_isServer ? "server" : "client"),
        ret, ToString().data());

  TimerManager::Instance()->AddTimer(&dtlsTimer_);

  return 0;
}

void DtlsHandler::SetStreamDirect(StreamDirection direct) {
  m_StmDirect = direct;
}

extern int g_sock_fd;

void DtlsHandler::SendToDtls(const void* data, int len) {
  SSL_write(mSsl, data, len);
}

int DtlsHandler::WriteDtlsPacket(const void* data, size_t len) {
  int ret = 0;

  m_CheckTime = g_now_ms;

  if (m_ResetFlag) {
    m_SendBuffNum = 0;
    m_ResetFlag = false;
    m_ReSendTime = 0;
  }

  if (MAX_BUFF_NUM > m_SendBuffNum && MAX_DTLS_PKG_LEN > len) {
    m_SendBuff[m_SendBuffNum].len = len;
    memcpy(m_SendBuff[m_SendBuffNum].buff, data, len);
    m_SendBuffNum++;
  } else {
    m_SendBuffNum = 0;
    m_ResetFlag = false;
  }

  // tylog("write Dtls message len %zu, MTU %u sslMtu:%d %s", len, DTLS_MTU,
  // mSsl->d1->mtu, ToString().data());
  tylog("write Dtls message len %zu, MTU %u %s", len, DTLS_MTU,
        ToString().data());

  // to avoid copy
  std::vector<char> bufToSend(static_cast<const char*>(data),
                              static_cast<const char*>(data) + len);
  DumpSendPacket(bufToSend);
  ret = belongingPC_.SendToClient(bufToSend);
  if (ret) {
    tylog("send to client ret=%d.", ret);

    return ret;
  }

  return 0;
}

int DtlsHandler::rewriteDtlsPacket(const void* data, size_t len) {
  int ret = 0;

  m_CheckTime = g_now_ms;

  if (nullptr != mSsl) {
    // tylog("ReWrite Dtls message len %zu, MTU %u sslMtu:%d %s", len, DTLS_MTU,
    // mSsl->d1->mtu, ToString().data());
    tylog("ReWrite Dtls message len %zu, MTU %u %s", len, DTLS_MTU,
          ToString().data());
  }

  // to avoid copy
  std::vector<char> bufToSend(static_cast<const char*>(data),
                              static_cast<const char*>(data) + len);
  DumpSendPacket(bufToSend);
  ret = belongingPC_.SendToClient(bufToSend);
  if (ret) {
    tylog("send to client ret=%d.", ret);

    return ret;
  }

  return 0;
}

// 在我方SDP中提供
std::string DtlsHandler::GetMyFingerprint() {
  assert(mCert != nullptr);  // init in constructor
  char fprint[MAX_FP_SIZE];
  memset(fprint, '\0', MAX_FP_SIZE);
  computeFingerprint(mCert, fprint);

  return std::string(fprint, strlen(fprint));
}

// @brief Get middle value
template <class T>
static inline T Clamp(const T& v, const T& lo, const T& hi) {
  return (v < lo) ? lo : (hi < v) ? hi : v;
}

int64_t DtlsHandler::GetCheckIntervalMs() const {
  return Clamp<int64_t>((MIN_RESEND_RTT * (m_ReSendTime + 1)),
                        MIN_RESEND_INTERVAL, MAX_RESEND_INTERVAL);
}

int DtlsHandler::OnTime() {
  int ret = 0;

  if (mHandshakeCompleted || mHandshakeFail) {
    return 0;
  }

  if (MAX_RESEND_TIME <= m_ReSendTime) {
    m_CheckTime = 0;
    m_ResetFlag = false;
    m_SSl_BuffState = SSL_ERROR_NONE;
    mHandshakeFail = true;
    tylog("max resend time reach max, dtls timeout, failed");

    return -1;
  }

  const int64_t TimePassMs = g_now_ms - m_CheckTime;
  const int64_t CheckIntervalMs = GetCheckIntervalMs();

  if ((CheckIntervalMs <= TimePassMs) && (0 != m_CheckTime) &&
      (!mHandshakeCompleted) && (SSL_ERROR_WANT_READ == m_SSl_BuffState) &&
      (0 != m_SendBuffNum)) {
    tylog("CheckIntervalMs=%" PRId64 ", TimePassMs=%" PRId64 " %s",
          CheckIntervalMs, TimePassMs, ToString().data());

    if (m_IsHandshakeCanComplete) {
      ++m_ClientKeySendTime;
    }
    ++m_ReSendTime;

    for (int i = 0; i < m_SendBuffNum; ++i) {
      tylog("onTime rewrite DtlsPacket, i=%d buffer len=%d", i,
            m_SendBuff[i].len);

      ret = rewriteDtlsPacket(m_SendBuff[i].buff, m_SendBuff[i].len);
      if (ret) {
        tylog("rewrite ret=%d", ret);

        return ret;
      }
    }

    tylog("after onTime rewrite, Now:%" PRId64 " %s passMs:%" PRId64 " %s",
          g_now_ms, tylib::MilliSecondToLocalTimeString(g_now_ms).data(),
          TimePassMs, ToString().data());
  }

  if (m_IsHandshakeCanComplete &&
      MAX_CLIENT_KEY_RESEND_TIME <= m_ClientKeySendTime) {
    tylog("Do handshakeCompleted time out, %s", ToString().data());

    const bool kSessionCompleted = true;
    ret = HandshakeCompleted(kSessionCompleted);
    if (ret) {
      tylog("handshakeCompleted fail, ret=%d", ret);

      return ret;
    }
  }

  return 0;
}

std::string DtlsHandler::ToString() const {
  return tylib::format_string(
      "{mSsl state=%d [%s], as %s, mHandshakeCompleted=%d, "
      "mGetKeyFlag=%d, m_SendBuffNum=%d, "
      "m_CheckTime Ms=%llu [%s], m_SSl_BuffState=%d [%s], "
      "m_ResetFlag=%d, m_ReSendTime=%d, mHandshakeFail=%d, "
      "m_ClientKeySendTime=%d, "
      "m_StmDirect=%s, m_IsHandshakeCanComplete=%d, "
      "m_LastSslState=%d [%s], m_startFlag=%d}",
      // 构造函数中确保mSsl不为空指针
      SSL_get_state(mSsl), SSL_state_string_long(mSsl),
      (m_isServer ? "server" : "client"), mHandshakeCompleted, mGetKeyFlag,
      m_SendBuffNum, m_CheckTime,
      tylib::MilliSecondToLocalTimeString(m_CheckTime).data(), m_SSl_BuffState,
      GetSslBuffStateString(m_SSl_BuffState), m_ResetFlag, m_ReSendTime,
      mHandshakeFail, m_ClientKeySendTime,
      GetStreamDirectionString(m_StmDirect), m_IsHandshakeCanComplete,
      m_LastSslState, GetSslStateString(m_LastSslState).data(), m_startFlag);
}

}  // namespace tywebrtc
