// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_ICE_ICE_HANDLER_H_
#define SRC_ICE_ICE_HANDLER_H_

#include <string>
#include <vector>

namespace tywebrtc {

#define STUN_MSG_MAX_LEN (2048)
#define STUN_MAGIC (0x2112A442)
#define STUN_XOR_FINGERPRINT_MASK (0x5354554eL)
#define STUN_IPV4_ADDR_LEN (8)
#define STUN_MESSGAE_INTEGRITY_LEN (20)
#define STUN_FINGERPRINT_LEN (4)

#define CMD_STUN_BINDING_REQ (0x0001)  // RFC 5389
#define CMD_STUN_BINGING_RES (0x0101)  // RFC 5389

#define ATTRIBUTE_XOR_MAPPED_ADDR (0x0020)    // RFC 5389
#define ATTRIBUTE_USER_NAME (0x0006)          // RFC 5389
#define ATTRIBUTE_CONTROLLING (0x802A)        // RFC 5245
#define ATTRIBUTE_USE_CANDIDATE (0x0025)      // RFC 5245
#define ATTRIBUTE_PRIORITY (0x0024)           // RFC 5245
#define ATTRIBUTE_MESSGAE_INTEGRITY (0x0008)  // RFC 5389
#define ATTRIBUTE_FINGERPRINT (0x8028)        // RFC 5389

#define ATTRIBUTE_NETWORK_INFO (0xC057)  // draft-thatcher-ice-network-cost-00

#pragma pack(1)

typedef struct StunMsgHead {
  unsigned short StunMsgType;
  unsigned short MsgLen;
  unsigned int Cookie;
#define STUN_MSG_TRANSACTION_ID_LEN (12)
  char TsxId[STUN_MSG_TRANSACTION_ID_LEN];
} STUN_MSG_HEAD;

typedef struct StunMsgXORMappedAddress {
  unsigned short AttributeType;
  unsigned short Len;
  char Reserved;
  char ProtocolFamily;
  unsigned short Port;
  unsigned int Ip;
} STUN_XOR_MSG_MAPPED_V4_ADDRESS;

typedef struct StunMsgIntegrity {
  unsigned short AttributeType;
  unsigned short Len;
  unsigned char HmacSha1[20];
} STUN_MSG_INTEGRITY;

typedef struct StunMsgFingerPrint {
  unsigned short AttributeType;
  unsigned short Len;
  unsigned int CRC32;
} STUN_MSG_FINGERPRINT;

typedef struct StunMsgCommon {
  unsigned short AttributeType;
  unsigned short Len;
} STUN_MSG_COMMON;

#pragma pack()

#define LOC_USER_NAME_LEN (256)
#define REM_USER_NAME_LEN (256)
#define REM_USER_PWD_LEN (256)
#define LOC_USER_PWD_LEN 256

enum ICE_CAND_TYPE {
  ICE_CAND_TYPE_HOST,   // ICE host candidate. A host candidate represents the
                        // actual local transport address in the host.
  ICE_CAND_TYPE_SRFLX,  // ICE server reflexive candidate, which represents the
                        // public mapped address of the local address,
  // and is obtained by sending STUN Binding request from the host candidate to
  // a STUN server.
  ICE_CAND_TYPE_PRFLX,  // ICE peer reflexive candidate, which is the address as
                        // seen by peer agent during connectivity check.
  ICE_CAND_TYPE_RELAYED,  // ICE relayed candidate, which represents the address
                          // allocated in TURN server.
  ICE_CAND_TYPE_BUTT
};

inline int IceGetTypePrefix(ICE_CAND_TYPE type) {
  switch (type) {
    case ICE_CAND_TYPE_HOST:
      return 'H';

    case ICE_CAND_TYPE_SRFLX:
      return 'S';

    case ICE_CAND_TYPE_PRFLX:
      return 'P';

    case ICE_CAND_TYPE_RELAYED:
      return 'R';

    default:
      return 'U';
  }
}

struct ICEInfo {
  // self
  char LocalUfrag[LOC_USER_NAME_LEN + 1];    // 4 to 256
  char LocalPassWord[LOC_USER_PWD_LEN + 1];  // 22 to 256

  // peer
  char RemoteUfrag[REM_USER_NAME_LEN + 1];    // 4 to 256
  char RemotePassWord[REM_USER_PWD_LEN + 1];  // 22 to 256

  char Foundation[64];
  unsigned int Prio = 0;

  std::string remoteUsername;
};

class PeerConnection;

class IceHandler {
 public:
  explicit IceHandler(PeerConnection &pc);
  int HandleIcePacket(const std::vector<char> &vBufReceive);

 private:
  int CheckIcePacket(const std::vector<char> &vBufReceive);
  int HandleBindReq(const std::vector<char> &vBufReceive);
  int EncoderXORMappedAddress(char *pBuff, int Len);
  int DecodeStunBindingAttributesMsg_(const STUN_MSG_COMMON *pMsgComm,
                                      int LeftLen, bool *o_bUseCandidate);
  int EncoderMsgIntergrity(char *pMsgIntergrityBuff, int LeftLen,
                           char *pHeadBuf, int Len);
  int EncoderFingerprint(const char *pFingerprintBuff, int FingerprintLen,
                         char *pHeadBuf, int Len);

  void CreatLocalUserStunInfo();
  int CreatUserFoundation();
  unsigned int IceCalcCandPrio(int type, unsigned int CompId);
  void CreatUserPrio();

 public:  // tmp
  PeerConnection &belongingPeerConnection_;

  ICEInfo iceInfo_;
};

}  // namespace tywebrtc

#endif  // SRC_ICE_ICE_HANDLER_H_
