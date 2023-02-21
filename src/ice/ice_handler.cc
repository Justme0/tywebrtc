#include "ice/ice_handler.h"

#include <arpa/inet.h>

#include <algorithm>
#include <cstring>

#include "tylib/ip/ip.h"

#include "log/log.h"
#include "openssl/hmac.h"
#include "pc/peer_connection.h"

// taylor to remove, should show RETURN in naming!
/*条件满足带返回值返回失败*/
#define WEB_ERROR_CHECK(cond, errid, fmt, args...) \
  do {                                             \
    if (cond) {                                    \
      tylog(fmt, ##args);                          \
      return errid;                                \
    }                                              \
  } while (0)

extern int g_sock_fd;
uint64_t g_tinyid = 144115262675375852;  // taylor to opt

#define STUN_HEX_DIGITS ("0123456789abcdef")

inline void ValToHexDigit(unsigned value, char *p) {
  *p++ = STUN_HEX_DIGITS[(value & 0xF0) >> 4];
  *p = STUN_HEX_DIGITS[(value & 0x0F)];
}

char *CreatRandString(char *str, int len) {
  char *p = str;
  int i = 0;

  for (i = 0; i < len / 8; ++i) {
    unsigned int val = rand();
    ValToHexDigit((val & 0xFF000000) >> 24, p + 0);
    ValToHexDigit((val & 0x00FF0000) >> 16, p + 2);
    ValToHexDigit((val & 0x0000FF00) >> 8, p + 4);
    ValToHexDigit((val & 0x000000FF) >> 0, p + 6);
    p += 8;
  }

  for (i = i * 8; i < len; ++i) {
    *p++ = STUN_HEX_DIGITS[rand() & 0x0F];
  }
  return str;
}

IceHandler::IceHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {
  CreatLocalUserStunInfo();
  CreatUserFoundation();
  CreatUserPrio();
}

void IceHandler::CreatLocalUserStunInfo() {
  /*用户初始化的时候生成本用户对应的用户名和密码*/
  std::string strTinyId = std::to_string(g_tinyid);
  int strlen = strTinyId.length();
  strncpy(iceInfo_.LocalUfrag, strTinyId.c_str(), strlen);
  iceInfo_.LocalUfrag[strlen] = '\0';

  // CreatRandString(iceInfo_.LocalPassWord,24);
  // iceInfo_.LocalPassWord[24] = '\0';
  strcpy(iceInfo_.LocalPassWord, "728c90257276ed9e48aac959");  // taylor
  tylog("ufrag=%s, pws=%s", iceInfo_.LocalUfrag, iceInfo_.LocalPassWord);
}

// https://datatracker.ietf.org/doc/html/rfc8445
// 5.1.1.3.  Computing Foundations
//
//    The ICE agent assigns each candidate a foundation.  Two candidates
//    have the same foundation when all of the following are true:
//
//    o  They have the same type (host, relayed, server reflexive, or peer
//       reflexive).
//
//    o  Their bases have the same IP address (the ports can be different).
//
//    o  For reflexive and relayed candidates, the STUN or TURN servers
//       used to obtain them have the same IP address (the IP address used
//       by the agent to contact the STUN or TURN server).
//
//    o  They were obtained using the same transport protocol (TCP, UDP).
//
//    Similarly, two candidates have different foundations if their types
//    are different, their bases have different IP addresses, the STUN or
//    TURN servers used to obtain them have different IP addresses (the IP
//    addresses used by the agent to contact the STUN or TURN server), or
//    their transport protocols are different.
int IceHandler::CreatUserFoundation() {
  unsigned int LocalOuterIP =
      tylib::stringToHostOrder("33.2.3.2");  // taylor tmp, now not use

  snprintf(iceInfo_.Foundation, sizeof(iceInfo_.Foundation), "%c%x",
           IceGetTypePrefix(ICE_CAND_TYPE_SRFLX), LocalOuterIP);

  iceInfo_.Foundation[63] = '\0';
  tylog("foundation=0x%s", iceInfo_.Foundation);

  return 0;
}

unsigned int IceHandler::IceCalcCandPrio(int type, unsigned int CompId) {
  static unsigned char CandTypePrefs[ICE_CAND_TYPE_BUTT] = {126, 100, 110, 0};

  return ((CandTypePrefs[type] & 0xFF) << 24) + ((65535 & 0xFFFF) << 8) +
         (((256 - CompId) & 0xFF) << 0);
}

// TODO simplify
void IceHandler::CreatUserPrio() {
  iceInfo_.Prio = IceCalcCandPrio(ICE_CAND_TYPE_SRFLX, 1);
}

// always return 0, should return error for wrong packet
// todo parameter can be std::span or std::string_view
int IceHandler::DecodeStunBindingAttributesMsg_(const STUN_MSG_COMMON *pMsgComm,
                                                int LeftLen,
                                                bool *o_bUseCandidate) {
  const int kStunAttributeCommonFieldLength = 4;  // attr type 2B, len 2B
  while (kStunAttributeCommonFieldLength <= LeftLen) {
    // TLV structure
    unsigned short AttributeType = ntohs(pMsgComm->AttributeType);
    int AttributeLen = ntohs(pMsgComm->Len);
    char *pData = (char *)pMsgComm + kStunAttributeCommonFieldLength;

    // opt: print readable
    tylog("AttributeType=%hu, attr len=%d, pData=%p", AttributeType,
          AttributeLen, pData);

    switch (AttributeType) {
      case ATTRIBUTE_USER_NAME: {    // todo use enum
        const int kEnoughLen = 513;  // input must not too long

        if (AttributeLen > kEnoughLen) {
          // log warning
          // monitro
        }

        std::string username(pData, std::min(kEnoughLen, AttributeLen));
        tylog("ice username=%s",
              username.data());  // taylor last char is '\0' ?

        // TODO check username

        break;
      }

      case ATTRIBUTE_CONTROLLING: {
        // char Controlling[32] = {'\0'};
        // m_pStunIceControlling = pMsgComm;
        std::string controlling(pData, 8);
        break;
      }

      case ATTRIBUTE_USE_CANDIDATE: {
        // important!
        // m_pStunUseCandidate = pMsgComm;
        *o_bUseCandidate = true;
        tylog("recv candidate");
        break;
      } /*unsigned int*/

      case ATTRIBUTE_PRIORITY: {
        // m_pStunPritority = pMsgComm;
        break;
      } /*hmac 20*/

      case ATTRIBUTE_MESSGAE_INTEGRITY: {
        // char Hmac[41] = {'\0'};
        // m_pStunMsgIntegrity = pMsgComm;

        // if (20 != AttributeLen) {
        // taylor m_LogStr << " MESSGAE_INTEGRITY:    Len=" << AttributeLen <<
        // " Err\n";
        // } else {
        // std::string Hmac(pData, 20);
        // }

        break;
      }

      case ATTRIBUTE_FINGERPRINT: {
        // m_pStunFingerprint = pMsgComm;
        std::string Fingerprint(pData, 4);
        break;
      }

      default: {
        // todo log

        break;
      }
    }

    /*长度只标记实际value长度，真个结构长度四字节对齐，后面可能会有padding数据*/
    AttributeLen = (AttributeLen + 3) & (~3);

    /*next attr's type+len*/
    if (AttributeLen + static_cast<int>(sizeof(STUN_MSG_COMMON)) >= LeftLen) {
      pMsgComm = reinterpret_cast<const STUN_MSG_COMMON *>(
          reinterpret_cast<const char *>(pMsgComm) + LeftLen);
      LeftLen = 0;
    } else {
      pMsgComm = reinterpret_cast<const STUN_MSG_COMMON *>(
          reinterpret_cast<const char *>(pMsgComm) + AttributeLen +
          sizeof(STUN_MSG_COMMON));
      LeftLen -= AttributeLen + sizeof(STUN_MSG_COMMON);
    }
  }

  return 0;
}

// taylor return not error code
int IceHandler::EncoderXORMappedAddress(char *pBuff, int Len) {
  // taylor should check Len
  (void)Len;
  STUN_XOR_MSG_MAPPED_V4_ADDRESS *pMapAddr =
      reinterpret_cast<STUN_XOR_MSG_MAPPED_V4_ADDRESS *>(pBuff);

  pMapAddr->AttributeType = htons(ATTRIBUTE_XOR_MAPPED_ADDR);
  pMapAddr->Len = htons(STUN_IPV4_ADDR_LEN);
  pMapAddr->Reserved = 0;
  pMapAddr->ProtocolFamily = 0x01;
  pMapAddr->Port =
      htons(belongingPeerConnection_.clientPort_) ^ htons(STUN_MAGIC >> 16);
  pMapAddr->Ip =
      inet_addr(belongingPeerConnection_.clientIP_.data()) ^ htonl(STUN_MAGIC);

  return (int)sizeof(STUN_XOR_MSG_MAPPED_V4_ADDRESS);
}

int IceHandler::EncoderMsgIntergrity(char *pMsgIntergrityBuff, int LeftLen,
                                     char *pHeadBuf, int Len) {
  WEB_ERROR_CHECK((nullptr == pMsgIntergrityBuff), -1, "pBuff is null");
  WEB_ERROR_CHECK(((int)sizeof(STUN_MSG_INTEGRITY) > LeftLen), -2,
                  "LeftLen[%d] err", LeftLen);
  WEB_ERROR_CHECK((nullptr == pHeadBuf), -3, "pHeadBuf is null");
  WEB_ERROR_CHECK((0 >= Len), -4, "Msg Len[%d] err", Len);
  STUN_MSG_INTEGRITY *pMsgIntergrity = (STUN_MSG_INTEGRITY *)pMsgIntergrityBuff;
  pMsgIntergrity->AttributeType = htons(ATTRIBUTE_MESSGAE_INTEGRITY);
  pMsgIntergrity->Len = htons(STUN_MESSGAE_INTEGRITY_LEN);

  tylog("OPENSSL_VERSION_NUMBER=%ld=0x%lx", OPENSSL_VERSION_NUMBER,
        OPENSSL_VERSION_NUMBER);

  //做HMAC-SHA1加密，生成加密串
  unsigned int Outlen = STUN_MESSGAE_INTEGRITY_LEN;

#if (OPENSSL_VERSION_NUMBER > 0x10100000L)
  HMAC_CTX *ctx = HMAC_CTX_new();
  WEB_ERROR_CHECK((nullptr == ctx), -5, "HMAC_CTX_new err");
  int ret = 0;
  const EVP_MD *evp_md = EVP_sha1();

  if (nullptr == evp_md) {
    HMAC_CTX_free(ctx);
    return -6;
  }

  ret = HMAC_Init_ex(ctx, (unsigned char *)iceInfo_.LocalPassWord,
                     strlen(iceInfo_.LocalPassWord), evp_md, nullptr);

  if (0 == ret) {
    HMAC_CTX_free(ctx);
    return -7;
  }

  ret = HMAC_Update(ctx, reinterpret_cast<uint8_t *>(pHeadBuf), Len);

  if (0 == ret) {
    HMAC_CTX_free(ctx);
    return -8;
  }

  HMAC_Final(ctx, reinterpret_cast<uint8_t *>(pMsgIntergrity->HmacSha1),
             &Outlen);
  HMAC_CTX_free(ctx);
#else
  HMAC_CTX ctx;
  HMAC_CTX_init(&ctx);  // old api
  tylog("iceInfo_.LocalPassWord=%s, strlen=%zu", iceInfo_.LocalPassWord,
        strlen(iceInfo_.LocalPassWord));
  HMAC_Init_ex(&ctx, iceInfo_.LocalPassWord, strlen(iceInfo_.LocalPassWord),
               EVP_sha1(), nullptr);  // should check EVP_sha1() return value
  HMAC_Update(&ctx, (unsigned char *)pHeadBuf, Len);
  HMAC_Final(&ctx, (unsigned char *)(pMsgIntergrity->HmacSha1), &Outlen);
  HMAC_CTX_cleanup(&ctx);  // old api
#endif

  return sizeof(STUN_MSG_INTEGRITY);
}

typedef struct Crc32Context { unsigned int CrcState; } CRC32_CTX;

void Crc32Init(CRC32_CTX *ctx) { ctx->CrcState = 0; }

#define CRC32_NEGL 0xffffffffL

static const unsigned int Crc32Tab[] = {
    0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L,
    0x706af48fL, 0xe963a535L, 0x9e6495a3L, 0x0edb8832L, 0x79dcb8a4L,
    0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L,
    0x90bf1d91L, 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
    0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L, 0x136c9856L,
    0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L,
    0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L,
    0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
    0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L,
    0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L, 0x26d930acL, 0x51de003aL,
    0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L,
    0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
    0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL, 0x76dc4190L,
    0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL,
    0x9fbfe4a5L, 0xe8b8d433L, 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL,
    0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
    0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL,
    0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L, 0x12b7e950L,
    0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L,
    0xfbd44c65L, 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
    0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL, 0x4369e96aL,
    0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L,
    0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL, 0xbe0b1010L,
    0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
    0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L,
    0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L, 0x9abfb3b6L,
    0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L,
    0x73dc1683L, 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
    0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L, 0xf00f9344L,
    0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL,
    0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL,
    0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
    0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L,
    0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL, 0xd80d2bdaL, 0xaf0a1b4cL,
    0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL,
    0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
    0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL, 0xc5ba3bbeL,
    0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L,
    0x2cd99e8bL, 0x5bdeae1dL, 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL,
    0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
    0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL,
    0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L, 0xf1d4e242L,
    0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L,
    0x18b74777L, 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
    0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L, 0xa00ae278L,
    0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L,
    0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L,
    0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
    0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L,
    0xcdd70693L, 0x54de5729L, 0x23d967bfL, 0xb3667a2eL, 0xc4614ab8L,
    0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL,
    0x2d02ef8dL};
#define CRC32_CALC_INDEX(c) (c & 0xff)
#define CRC32_SHIFTE(c) (c >> 8)
unsigned int Crc32Update(CRC32_CTX *ctx, const unsigned char *data,
                         int nbytes) {
  unsigned int crc = ctx->CrcState ^ CRC32_NEGL;

  for (; (((unsigned long)(long)data) & 0x03) && nbytes > 0; --nbytes) {
    crc = Crc32Tab[CRC32_CALC_INDEX(crc) ^ *data++] ^ CRC32_SHIFTE(crc);
  }

  while (nbytes >= 4) {
    crc ^= *(const unsigned int *)data;
    crc = Crc32Tab[CRC32_CALC_INDEX(crc)] ^ CRC32_SHIFTE(crc);
    crc = Crc32Tab[CRC32_CALC_INDEX(crc)] ^ CRC32_SHIFTE(crc);
    crc = Crc32Tab[CRC32_CALC_INDEX(crc)] ^ CRC32_SHIFTE(crc);
    crc = Crc32Tab[CRC32_CALC_INDEX(crc)] ^ CRC32_SHIFTE(crc);
    nbytes -= 4;
    data += 4;
  }

  while (nbytes--) {
    crc = Crc32Tab[CRC32_CALC_INDEX(crc) ^ *data++] ^ CRC32_SHIFTE(crc);
  }

  ctx->CrcState = crc ^ CRC32_NEGL;

  return ctx->CrcState;
}

unsigned int Crc32Final(CRC32_CTX *ctx) { return ctx->CrcState; }

unsigned int Crc32Calc(const unsigned char *data, int nbytes) {
  CRC32_CTX ctx;

  Crc32Init(&ctx);
  Crc32Update(&ctx, data, nbytes);
  return Crc32Final(&ctx);
}

int IceHandler::EncoderFingerprint(const char *pFingerprintBuff,
                                   int FingerprintLen, char *pHeadBuf,
                                   int Len) {
  WEB_ERROR_CHECK((nullptr == pFingerprintBuff), -1, "pBuff is null");
  WEB_ERROR_CHECK(((int)sizeof(STUN_MSG_FINGERPRINT) > FingerprintLen), -2,
                  "Len[%d] err", FingerprintLen);
  STUN_MSG_FINGERPRINT *pMsgIntergrity =
      (STUN_MSG_FINGERPRINT *)pFingerprintBuff;
  pMsgIntergrity->AttributeType = htons(ATTRIBUTE_FINGERPRINT);
  pMsgIntergrity->Len = htons(STUN_FINGERPRINT_LEN);

  pMsgIntergrity->CRC32 = Crc32Calc((const unsigned char *)pHeadBuf, Len);
  pMsgIntergrity->CRC32 ^= STUN_XOR_FINGERPRINT_MASK;
  pMsgIntergrity->CRC32 = htonl(pMsgIntergrity->CRC32);
  return (int)sizeof(STUN_MSG_FINGERPRINT);
}

int IceHandler::HandleBindReq(const std::vector<char> &vBufReceive) {
  int ret = 0;

  bool bUseCandidate = false;
  ret = DecodeStunBindingAttributesMsg_(
      reinterpret_cast<const STUN_MSG_COMMON *>(vBufReceive.data() +
                                                sizeof(STUN_MSG_HEAD)),
      vBufReceive.size() - sizeof(STUN_MSG_HEAD), &bUseCandidate);
  // now always return 0
  if (0 != ret) {
    tylog("decodeStunBindingAttributesMsg fail, ret=%d", ret);

    return ret;
  }

  if (bUseCandidate &&
      EnumStateMachine::GOT_USE_CANDIDATE_ICE >
          belongingPeerConnection_.stateMachine_) {
    // When in communication, will always recv use-candiate ice, so check
    // machine state before assigning
    belongingPeerConnection_.stateMachine_ =
        EnumStateMachine::GOT_USE_CANDIDATE_ICE;
    tylog("GOT_USE_CANDIDATE_ICE, ice done, now start dtls ...");

    // 收到带UseCandidate属性的STUN包后启动DTLS
    ret = belongingPeerConnection_.dtlsHandler_.StartDTLS();
    if (ret) {
      tylog("dtls start fail, ret=%d", ret);

      return ret;
    }

    TimerManager::Instance()->AddTimer(
        &this->belongingPeerConnection_.dtlsTimer_);
  } else if (EnumStateMachine::GOT_FIRST_ICE >
             belongingPeerConnection_.stateMachine_) {
    belongingPeerConnection_.stateMachine_ = EnumStateMachine::GOT_FIRST_ICE;
    tylog("got first STUN");
  }

  /*回包*/
  char SndBuff[STUN_MSG_MAX_LEN];
  STUN_MSG_HEAD *pHeadReq = (STUN_MSG_HEAD *)vBufReceive.data();
  STUN_MSG_HEAD *pHeadRes = (STUN_MSG_HEAD *)SndBuff;
  pHeadRes->StunMsgType = ntohs(CMD_STUN_BINGING_RES);
  pHeadRes->Cookie = ntohl(STUN_MAGIC);
  std::copy(pHeadReq->TsxId, pHeadReq->TsxId + STUN_MSG_TRANSACTION_ID_LEN,
            pHeadRes->TsxId);

  // ok
  char *pOffset = SndBuff + sizeof(STUN_MSG_HEAD);
  int LeftLen = STUN_MSG_MAX_LEN - sizeof(STUN_MSG_HEAD);
  int EncLen = EncoderXORMappedAddress(pOffset, LeftLen);
  if (4 >= EncLen) {
    tylog("encoderXORMappedAddress enclen=%d", EncLen);
    return -1;
  }
  pOffset += EncLen;
  LeftLen -= EncLen;

  // ok
  /*根据 pre-RFC3489bis-07 以及之后的协议要求,计算MESSAGE-INTEGRITY和FINGERPRINT
     的时候，MESSAGE-INTEGRITY长度包含在计算中，FINGERPRINT的长度不包含在计算中，
     参考pjlib实现*/
  int MsgLen = EncLen + sizeof(STUN_MSG_INTEGRITY);
  pHeadRes->MsgLen = ntohs(MsgLen);
  int PartLen = (int)(pOffset - SndBuff);
  EncLen = EncoderMsgIntergrity(pOffset, LeftLen, SndBuff, PartLen);
  if (4 >= EncLen) {
    tylog("encoderMsgIntergrity enclen=%d", EncLen);
    return -2;
  }
  pOffset += EncLen;
  LeftLen -= EncLen;
  PartLen = (int)(pOffset - SndBuff);

  // ok
  MsgLen += sizeof(STUN_MSG_FINGERPRINT);
  pHeadRes->MsgLen = ntohs(MsgLen);
  EncLen = EncoderFingerprint(pOffset, LeftLen, SndBuff, PartLen);
  if (4 >= EncLen) {
    tylog("encoderFingerprint enclen=%d", EncLen);
    return -3;
  }
  pOffset += EncLen;
  LeftLen -= EncLen;
  int SndLen = (int)(pOffset - SndBuff);

  // to avoid copy
  std::vector<char> bufToSend(SndBuff, SndBuff + SndLen);
  ret = belongingPeerConnection_.SendToClient(bufToSend);
  if (ret) {
    tylog("send to client ret=%d.", ret);

    return ret;
  }

  return 0;
}

int IceHandler::CheckIcePacket(const std::vector<char> &vBufReceive) {
  const char *buff = vBufReceive.data();
  int len = vBufReceive.size();

  WEB_ERROR_CHECK((len < static_cast<int32_t>(sizeof(StunMsgHead))), -1,
                  "buffer len=%d too small", len);
  WEB_ERROR_CHECK(
      (0x00 != *buff && 0x01 != *buff), -2,
      "first char is either 0x00(binding req) or 0x01(binding rsp)");
  const StunMsgHead *pHead = reinterpret_cast<const StunMsgHead *>(buff);
  int MsgLen = ntohs(pHead->MsgLen);
  // todo len var use signed int
  WEB_ERROR_CHECK(
      (MsgLen + static_cast<int32_t>(sizeof(StunMsgHead)) != len), -4,
      "MsgLen:%d(head stuct size)+%zu(means body len) must equal to "
      "%d(recv buf len)",
      MsgLen, sizeof(StunMsgHead), len);
  WEB_ERROR_CHECK(((MsgLen & 0x03) != 0), -5,
                  "body len(%d) should be times of 4", MsgLen);

  return 0;
}

int IceHandler::HandleIcePacket(const std::vector<char> &vBufReceive) {
  int ret = 0;

  ret = CheckIcePacket(vBufReceive);
  if (ret) {
    tylog("CheckIcePacket fail, ret=%d", ret);
    return ret;
  }

  const STUN_MSG_HEAD *pHead =
      reinterpret_cast<const STUN_MSG_HEAD *>(vBufReceive.data());

  if (EnumStateMachine::GOT_CANDIDATE >
      belongingPeerConnection_.stateMachine_) {
    return -1;
  }

  unsigned short MsgType = ntohs(pHead->StunMsgType);
  tylog("MsgType=%d", MsgType);

  switch (MsgType) {
    case CMD_STUN_BINDING_REQ: {
      ret = HandleBindReq(vBufReceive);
      if (ret) {
        tylog("HandleBindReq ret=%d", ret);
        return ret;
      }

      break;
    }

    case CMD_STUN_BINGING_RES: {
      break;
    }
    default: { break; }
  }

  return 0;
}
