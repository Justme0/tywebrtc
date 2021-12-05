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

#include "dtls/dtls_handler.h"

#include <cassert>

#include "tylib/codec/codec.h"

#include "dtls/certificate_key.h"
#include "log/log.h"
#include "openssl/bio.h"
#include "pc/peer_connection.h"

// taylor to use c++20 std::format
// https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
static inline std::string format_string(const char* szFmt, ...)
    __attribute__((format(printf, 1, 2)));
static inline std::string format_string(const char* szFmt, ...) {
  int n = 0;

  va_list ap;

  // max string allowed
  const int iLargeSize = 65536;
  static char szLargeBuff[iLargeSize];

  va_start(ap, szFmt);
  n = vsnprintf(szLargeBuff, sizeof(szLargeBuff), szFmt, ap);
  va_end(ap);

  if (n >= iLargeSize) {
    n = iLargeSize - 1;
  }

  return std::string(szLargeBuff, n);
}

int64_t g_GetNowMs() {
  // taylor : move get now time util to tylib
  // * time
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);

  // get sec(tm struct presentation), ms, us
  struct tm tm;
  localtime_r(&t.tv_sec, &tm);

  int ms = t.tv_nsec / 1000000;
  return t.tv_sec + ms;
}

const int SRTP_MASTER_KEY_KEY_LEN = 16;
const int SRTP_MASTER_KEY_SALT_LEN = 14;
const int DTLS_MTU = 1100;

const char* DtlsHandler::DefaultSrtpProfile = "SRTP_AES128_CM_SHA1_80";
X509* DtlsHandler::mCert = NULL;  //证书格式，包含公钥、加密算法、有效期等参数。
EVP_PKEY* DtlsHandler::privkey = NULL;  // key 的EVP
                                        // 封装可封装rsa、dsa、ecc等key

std::string WebRtcPrintTimeMs(unsigned long long TimeMs) {
  // taylor TODO
  return "taylorMock";
}

int DummyCb(int preverify_ok, X509_STORE_CTX* x509_ctx) { return 1; }

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
      return "SSL_ERROR_SYSCALL";  // look at error stack/return value/errno
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
#endif
}

// static inline int dummy_cb(int d, X509_STORE_CTX* x) { return 1; }

void SSLInfoCallback(const SSL* s, int where, int ret) {
  if (where & SSL_CB_HANDSHAKE_DONE) {
    int ret = SSL_get_verify_result(s);

    tylog("SSL_get_verify_result: OK[%d]", ret);
    tylog("SSL_CB_HANDSHAKE_DONE ret=%d, ssl state=%d[%s]", ret,
          SSL_get_state(s), SSL_state_string_long(s));
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
    tylog("%s", SSL_state_string_long(s));
  } else if (where & SSL_CB_ALERT) {
    str = (where & SSL_CB_READ) ? "read" : "write";
    tylog("SSL3[State %s] alert %d - %s; %s : %s", strState, ret, str,
          SSL_alert_type_string_long(ret), SSL_alert_desc_string_long(ret));

    tylog("SSL3[State %s] where=%d[%s], alert ret=%d(0x%X)[type=%s, desc=%s]",
          strState,

          where, str,

          ret, ret, SSL_alert_type_string_long(ret),
          SSL_alert_desc_string_long(ret));
  } else if (where & SSL_CB_EXIT) {
    if (ret == 0) {
      tylog("failed[%d] in %s", ret, SSL_state_string_long(s));
    } else if (ret < 0) {
      tylog("error[%d] in %s", ret, SSL_state_string_long(s));
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

    tylog("Error: %d", X509_V_ERR_DEPTH_ZERO_SELF_SIGNED_CERT);

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
                        long argl, long ret) {
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
      d->WriteDtlsPacket(argp, argi);
    }
  }

  return r;
}

void DtlsHandler::InitOpensslAndCert() {
  tylog("shit ok");
#if (OPENSSL_VERSION_NUMBER > 0x10100000L)
  OPENSSL_init_ssl(0, NULL);
  tylog("shit 2");
  OPENSSL_init_crypto(OPENSSL_INIT_ADD_ALL_CIPHERS, NULL);
  tylog("shit ");
  OPENSSL_init_crypto(OPENSSL_INIT_ADD_ALL_DIGESTS, NULL);
  tylog("shit ");
#else
  tylog("shit ok");
  SSL_library_init();
  tylog("shit ok");
  SSL_load_error_strings();
  tylog("shit ok");
  ERR_load_crypto_strings();
  tylog("shit ok");
#endif
  int ret = GetCertificateAndKey(DtlsHandler::mCert, DtlsHandler::privkey);
  if (ret) {
    tylog("getCertificateAndKey shit");
  }
}

DtlsHandler::DtlsHandler(PeerConnection& pc, bool isServer)
    : belongingPeerConnection_(pc),
      m_isServer(isServer),
      mHandshakeCompleted(false),
      mGetKeyFlag(false),
      m_SendBuffNum(0),
      m_CheckTime(g_GetNowMs()),
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
      m_startFlag(false) {
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
  assert(mSsl != NULL);
  SSL_set_mtu(mSsl, DTLS_MTU);
  mInBio = BIO_new(BIO_s_mem());

  mOutBio = BIO_new(BIO_s_mem());
  BIO_ctrl(mOutBio, BIO_CTRL_DGRAM_SET_MTU, 0, NULL);
  BIO_set_callback(mOutBio, DtlsOutBIOCallback);
  BIO_set_callback_arg(mOutBio, reinterpret_cast<char*>(this));

  SSL_set_bio(mSsl, mInBio, mOutBio);
}

DtlsHandler::~DtlsHandler() {
  tylog("in destructor closing dtls, %s", ToString().data());

  if (mSsl != NULL) {
    // SSL_shutdown(mSsl);
    SSL_free(mSsl);
    mSsl = NULL;
  }

  SSL_CTX_free(mContext);
}

void DtlsHandler::onHandshakeCompleted() {
  tylog("isServer: %d clientKey: %s, serverKey: %s, swap keys if server",
        m_isServer, sendingRtpKey.c_str(), receivingRtpKey.c_str());

  if (m_isServer) {
    // If we are server, we swap the keys
    tylog("message: swapping keys, isServer: %d", m_isServer);
    sendingRtpKey.swap(receivingRtpKey);
  }

  bool ok = belongingPeerConnection_.srtpHandler_.SetRtpParams(sendingRtpKey,
                                                               receivingRtpKey);
  if (!ok) {
    tylog("NOTE!!! srtpHandler_.SetRtpParams fail, but return is void :(");
  }

  belongingPeerConnection_.stateMachine_ = EnumStateMachine::DTLS_DONE;
  tylog("DTLS_DONE, message:HandShakeCompleted");
}

// 目前实参都是true，再次确认why？
// should return err?
void DtlsHandler::HandshakeCompleted(bool bSessionCompleted) {
  if (mGetKeyFlag) {
    // tylog("We get Key, no need to do it, bSessionCompleted=%d, %s",
    // bSessionCompleted, ToString().data());
    mHandshakeCompleted = bSessionCompleted;

    return;
  }

  char fprint[MAX_FP_SIZE];
  memset(fprint, '\0', MAX_FP_SIZE);

  if (!getRemoteFingerprint(fprint)) {
    tylog("Peer did not authenticate %s", ToString().data());
    return;
  }

  bool checkOk = checkFingerprint(fprint, strlen(fprint));
  if (!checkOk) {
    // 两次获取对端指纹，应当相同
    tylog("check fingerprint fail %s", ToString().data());
    return;
  }

  // const int SRTP_MASTER_KEY_LEN = 30;
  unsigned char material[SRTP_MASTER_KEY_LEN << 1];

  // SSL_export_keying_material() returns 0 or -1 on failure or 1 on success.
  int ret = SSL_export_keying_material(mSsl, material, sizeof(material),
                                       "EXTRACTOR-dtls_srtp", 19, NULL, 0, 0);
  if (1 != ret) {
    tylog("SSL_export_keying_material err, ret=%d %s", ret, ToString().data());
    return;
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
  // taylor to escape copy string
  sendingRtpKey = tylib::Base64Encode(
      std::string(cKey, SRTP_MASTER_KEY_KEY_LEN + SRTP_MASTER_KEY_SALT_LEN));
  receivingRtpKey = tylib::Base64Encode(
      std::string(sKey, SRTP_MASTER_KEY_KEY_LEN + SRTP_MASTER_KEY_SALT_LEN));

  tylog("ClientKey: %s", sendingRtpKey.c_str());
  tylog("ServerKey: %s", receivingRtpKey.c_str());

  SRTP_PROTECTION_PROFILE* srtp_profile = SSL_get_selected_srtp_profile(mSsl);
  if (srtp_profile) {
    tylog("SRTP Extension negotiated profile=%s", srtp_profile->name);
  }

  mGetKeyFlag = true;  // 本变量只在此处修改

  if (bSessionCompleted) {
    mHandshakeCompleted = true;
  }

  tylog("bSessionCompleted=%d %s", bSessionCompleted, ToString().data());

  onHandshakeCompleted();
}

// 只在处理DTLS包结尾处调用
void DtlsHandler::CheckHandshakeComplete() {
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
    HandshakeCompleted(true /* session complete */);
  }

  if (m_isServer) {
    return;
  }

  if (!m_IsHandshakeCanComplete) {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
    m_IsHandshakeCanComplete =
        (SSL3_ST_CR_SESSION_TICKET_A == SSL_get_state(mSsl) ||
         SSL3_ST_CR_FINISHED_A == SSL_get_state(mSsl));
#else
    m_IsHandshakeCanComplete = (TLS_ST_CW_FINISHED == SSL_get_state(mSsl));
#endif
  }

  tylog("as client, %s", ToString().data());
  if (m_IsHandshakeCanComplete) {
    tylog("Do handshakeCompleted CW_FINISHED");
    HandshakeCompleted(true);  // should verify?
  }
}

// @brief 收到DTLS包处理
// @param [in] vBufReceive DTLS包
// @return 获取到秘钥返true，否则返false
// 是否合理，错误是否要表示出来返回？taylor
bool DtlsHandler::HandleDtlsPacket(const std::vector<char>& vBufReceive) {
  tylog("read Dtls message len:%zu %s", vBufReceive.size(), ToString().data());

  if (!m_startFlag && m_isServer) {
    tylog("as DTLS server, recv DTLS packet before user-candidate STUN");

    StartDTLS();
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
    for (int i = 0; i < m_SendBuffNum; ++i) {
      tylog("already complete, rewriteDtlsPacket i=%d, buffer len=%d", i,
            m_SendBuff[i].len);
      rewriteDtlsPacket(m_SendBuff[i].buff, m_SendBuff[i].len);
    }
    if (0 != m_SendBuffNum) {
    } else {
      tylog("handshake completed, last sent package should exist %s",
            ToString().data());
    }

    return mGetKeyFlag;
  }

  m_ResetFlag = true;
  int r = 0;
  r = BIO_reset(mInBio);  // todo handle return value
  r = BIO_reset(mOutBio);
  r = BIO_write(mInBio, vBufReceive.data(), vBufReceive.size());
  if (r != static_cast<int>(vBufReceive.size())) {
    tylog("error BIO_write() r=%d, len=%zu, should be equal, %s", r,
          vBufReceive.size(), ToString().data());
    // taylor error handle ?
  }
  assert(r == static_cast<int>(vBufReceive.size()));  // 是否返回
  CheckHandshakeComplete();

  return mGetKeyFlag;
}

// @beief 获取认证对应的指纹
// @param [in] cert 认证
// @param [out] fingerprint 指纹
//
// optimize: check buffer size, return std::string
void DtlsHandler::computeFingerprint(X509* cert, char* fingerprint) const {
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
// @param [out] fprint 指纹
// @return 是否成功获取指纹
bool DtlsHandler::getRemoteFingerprint(char* fprint) const {
  X509* x = SSL_get_peer_certificate(mSsl);
  if (NULL == x) {
    tylog("SSL_get_peer_certificate NULL %s", ToString().data());
    return false;
  }

  computeFingerprint(x, fprint);
  X509_free(x);

  return true;
}

// @brief 比较对端指纹和入参
// @return 是否一致
bool DtlsHandler::checkFingerprint(const char* fingerprint,
                                   unsigned int len) const {
  char fprint[MAX_FP_SIZE];

  if (!getRemoteFingerprint(fprint)) {
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

// should return int?
void DtlsHandler::StartDTLS() {
  if (m_startFlag) {
    tylog("already start DTLSRTP %s", ToString().data());

    return;
  }
  m_startFlag = true;

  int ret = 0;

  tylog("enter startDTLS(), %s", ToString().data());

  if (m_isServer) {
    SSL_set_accept_state(mSsl);
    SSL_set_verify(mSsl, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT,
                   DummyCb);

    ret = SSL_do_handshake(mSsl);
    m_SSl_BuffState = SSL_get_error(mSsl, ret);

  } else {
    SSL_set_connect_state(mSsl);
    tylog("dtls as client, GetKeyFlag=%d", mGetKeyFlag);

    if (mGetKeyFlag) {
      return;
    }

    ret = SSL_accept(mSsl);
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
    ret = SSL_do_handshake(mSsl);
#endif
    m_SSl_BuffState = SSL_get_error(mSsl, ret);
  }

  tylog("Start DTLSRTP as %s, ret=%d, %s", (m_isServer ? "server" : "client"),
        ret, ToString().data());
}

void DtlsHandler::SetStreamDirect(StreamDirection direct) {
  m_StmDirect = direct;
}

bool DtlsHandler::GetHandshakeCompleted() const { return mHandshakeCompleted; }

extern int g_sock_fd;
extern struct sockaddr_in g_stConnAddr;  // to use data member

void DtlsHandler::WriteDtlsPacket(const void* data, size_t len) {
  m_CheckTime = g_GetNowMs();

  if (m_ResetFlag) {
    m_SendBuffNum = 0;
    m_ResetFlag = false;
    m_ReSendTime = 0;
  }

  if ((MAX_BUFF_NUM > m_SendBuffNum) && (MAX_DTLS_PKG_LEN > len)) {
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

  //    WebRtcDumpVideoPkg((char*)data,len,WEB_RTC_DUMP_ICE_OUT);

  // if (DropDownPkgRand())
  // {
  //     tylog("drop WRITE dtls pkg len:%d", len);
  //     return;
  // }
  // pUser->SendDtlsPacketToClient((const char*)data, len);
  ssize_t sendtoLen = sendto(g_sock_fd, data, len, 0,
                             reinterpret_cast<struct sockaddr*>(&g_stConnAddr),
                             sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    return;
  }
  tylog("sendto succ buf size=%ld", sendtoLen);
}

void DtlsHandler::rewriteDtlsPacket(const void* data, size_t len) {
  m_CheckTime = g_GetNowMs();

  if (NULL != mSsl) {
    // tylog("ReWrite Dtls message len %zu, MTU %u sslMtu:%d %s", len, DTLS_MTU,
    // mSsl->d1->mtu, ToString().data());
    tylog("ReWrite Dtls message len %zu, MTU %u %s", len, DTLS_MTU,
          ToString().data());
  }

  //    WebRtcDumpVideoPkg((char*)data,len,WEB_RTC_DUMP_ICE_OUT);

  // if (DropDownPkgRand())
  // {
  //     tylog("drop WRITE dtls pkg len:%d", len);
  //     return;
  // }
  // pUser->SendDtlsPacketToClient((const char*)data, len);
  ssize_t sendtoLen = sendto(g_sock_fd, data, len, 0,
                             reinterpret_cast<struct sockaddr*>(&g_stConnAddr),
                             sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    return;
  }
  tylog("sendto succ buf size=%ld", sendtoLen);
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

void DtlsHandler::OnTime() {
  if (mHandshakeCompleted || mHandshakeFail) {
    return;
  }

  if (MAX_RESEND_TIME <= m_ReSendTime) {
    m_CheckTime = 0;
    m_ResetFlag = false;
    m_SSl_BuffState = SSL_ERROR_NONE;
    mHandshakeFail = true;
    tylog("max resend time reach max, dtls timeout, failed");

    return;
  }

  const int64_t TimePassMs = g_GetNowMs() - m_CheckTime;
  const int64_t CheckIntervalMs = GetCheckIntervalMs();

  if ((CheckIntervalMs <= TimePassMs) && (0 != m_CheckTime) &&
      (!mHandshakeCompleted) && (SSL_ERROR_WANT_READ == m_SSl_BuffState) &&
      (0 != m_SendBuffNum)) {
    tylog("CheckIntervalMs=%ld, TimePassMs=%ld %s", CheckIntervalMs, TimePassMs,
          ToString().data());

    if (m_IsHandshakeCanComplete) {
      ++m_ClientKeySendTime;
    }
    ++m_ReSendTime;

    for (int i = 0; i < m_SendBuffNum; ++i) {
      tylog("OnTime rewriteDtlsPacket, i=%d buffer len=%d", i,
            m_SendBuff[i].len);

      rewriteDtlsPacket(m_SendBuff[i].buff, m_SendBuff[i].len);
    }

    tylog("after onTime rewrite, Now:%llu %s passMs:%ld %s", g_GetNowMs(),
          WebRtcPrintTimeMs(g_GetNowMs()).data(), TimePassMs,
          ToString().data());
  }

  if (m_IsHandshakeCanComplete &&
      (MAX_CLIENT_KEY_RESEND_TIME <= m_ClientKeySendTime)) {
    tylog("Do handshakeCompleted time out %s", ToString().data());

    HandshakeCompleted(true);
  }
}

std::string DtlsHandler::ToString() const {
  return format_string(
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
      m_SendBuffNum, m_CheckTime, WebRtcPrintTimeMs(m_CheckTime).data(),
      m_SSl_BuffState, GetSslBuffStateString(m_SSl_BuffState), m_ResetFlag,
      m_ReSendTime, mHandshakeFail, m_ClientKeySendTime,
      GetStreamDirectionString(m_StmDirect), m_IsHandshakeCanComplete,
      m_LastSslState, GetSslStateString(m_LastSslState).data(), m_startFlag);
}
