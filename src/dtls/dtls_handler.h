// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_DTLS_DTLS_HANDLER_H_
#define SRC_DTLS_DTLS_HANDLER_H_

#include <string>
#include <vector>

#include "openssl/crypto.h"
#include "openssl/err.h"
#include "openssl/ssl.h"

#include "src/timer/timer.h"

namespace tywebrtc {

#pragma pack(1)

typedef struct TagDtlsRecordLayer {
  char type;
  short version;
  short epoch;
  short SeqH;
  unsigned int SeqL;
  short Len;
} DTLS_RECORD_LAYER;

typedef struct TagDtlsAlert {
  char type;
  short version;
  short epoch;
  short SeqH;
  unsigned int SeqL;
  short Len;
  char AlertLevel;
  char Description;
} DTLS_ALERT;

#pragma pack()

#define MAX_DTLS_PKG_LEN (1500)
#define MAX_BUFF_NUM (10)
#define MAX_RESEND_TIME (64)
#define MAX_RESEND_INTERVAL (500)
#define MIN_RESEND_INTERVAL (50)
#define MIN_RESEND_RTT (50)
#define MAX_DTLS_NUM (1000)
#define MAX_CLIENT_KEY_RESEND_TIME (12)
#define MAX_FP_SIZE (128)

struct DtlsBuff {
  char buff[MAX_DTLS_PKG_LEN];
  int32_t len;
};

enum StreamDirection {
  kSendRecv,
  kSendOnly,
  kRecvOnly,
};

inline const char* GetStreamDirectionString(StreamDirection streamDirection) {
  switch (streamDirection) {
    case kSendRecv:
      return "kSendRecv";
    case kSendOnly:
      return "kSendOnly";
    case kRecvOnly:
      return "kRecvOnly";
    default:
      return "Unknown";
  }
}

class PeerConnection;

class DtlsHandler {
 public:
  explicit DtlsHandler(PeerConnection& pc, bool isServer);
  ~DtlsHandler();

  int StartDTLS();
  // bool IsGetKey() const { return mGetKeyFlag; }
  int HandleDtlsPacket(const std::vector<char>& vBufReceive);

  void SetStreamDirect(StreamDirection direct);
  int OnTime();
  std::string GetMyFingerprint();  // the fingerprint of the user cert
  void SendToDtls(const void* data, int len);
  int WriteDtlsPacket(const void* data, size_t len);
  std::string ToString() const;
  int HandshakeCompleted(bool bSessionComplete);

 private:
  int OnHandshakeCompleted_();
  bool CheckFingerprint_(const char* fingerprint, unsigned int len) const;
  bool GetRemoteFingerprint(char* fprint) const;
  void computeFingerprint(const X509* cert, char* fingerprint) const;
  void InitOpensslAndCert();
  int rewriteDtlsPacket(const void* data, size_t len);
  void CheckHandshakeComplete_();
  int64_t GetCheckIntervalMs() const;
  int DoDataChannel_(const std::vector<char>& vBufReceive);

 private:
  PeerConnection& belongingPC_;

  static const char* DefaultSrtpProfile;
  static X509* mCert;
  static EVP_PKEY* privkey;

  SSL* mSsl;
  bool m_isServer;  // todo rename: is dtls server
  SSL_CTX* mContext;
  BIO* mInBio;
  BIO* mOutBio;
  bool mHandshakeCompleted;
  bool mGetKeyFlag;
  std::string sendingRtpKey_;
  std::string receivingRtpKey_;

  // OPT: use vector
  DtlsBuff m_SendBuff[MAX_BUFF_NUM];  // 备份发送的数据包
  int m_SendBuffNum;

  unsigned long long m_CheckTime;  // 单位：毫秒
  int m_SSl_BuffState;
  bool m_ResetFlag;
  int m_ReSendTime;
  int mHandshakeFail;
  int32_t m_ClientKeySendTime;
  StreamDirection m_StmDirect;
  bool m_IsHandshakeCanComplete;
  int m_LastSslState;
  bool m_startFlag;
  DTLSTimer dtlsTimer_;
};

}  // namespace tywebrtc

#endif  // SRC_DTLS_DTLS_HANDLER_H_
