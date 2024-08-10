// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtmp/rtmp_assist.h"

#include <setjmp.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <sys/time.h>
#include <unistd.h>

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>

#include "colib/co_routine.h"
#include "librtmp/log.h"
#include "librtmp/rtmp_sys.h"
#include "openssl/md5.h"

#include "src/global_tmp/global_tmp.h"
#include "src/global_tmp/h264NaluDec.h"
#include "src/global_tmp/h264SpsDec.h"
#include "src/log/log.h"
#include "src/pc/peer_connection.h"
#include "src/pc/peer_connection_manager.h"
#include "src/rtmp/DomainResolve.h"
#include "src/rtp/pack_unpack/pack_unpack_common.h"

namespace tywebrtc {

#undef CRYPTO
typedef enum { RTMPT_OPEN = 0, RTMPT_SEND, RTMPT_IDLE, RTMPT_CLOSE } RTMPTCmd;
static const int packetSize[] = {12, 8, 4, 1};

void AV_erase(RTMP_METHOD* vals, int* num, int i, int freeit) {
  if (freeit) {
    free(vals[i].name.av_val);
  }

  (*num)--;

  for (; i < *num; i++) {
    vals[i] = vals[i + 1];
  }

  vals[i].name.av_val = NULL;
  vals[i].name.av_len = 0;
  vals[i].num = 0;
}

void AMF_Dump_Plus(AMFObject* obj, char* pStr, int* pOffset);

void AMFProp_Dump_Plus(AMFObjectProperty* prop, char* pStr, int* pOffset) {
  char strRes[256];
  char str[256];
  AVal name;

  if (prop->p_type == AMF_INVALID) {
    (*pOffset) +=
        snprintf(pStr + (*pOffset), 2048 - (*pOffset), "Property: INVALID\r\n");
    return;
  }

  if (prop->p_type == AMF_NULL) {
    (*pOffset) +=
        snprintf(pStr + (*pOffset), 2048 - (*pOffset), "Property: NULL\r\n");
    return;
  }

  if (prop->p_name.av_len) {
    name = prop->p_name;
  } else {
    // name.av_val = "no-name.";
    // name.av_len = sizeof("no-name.") - 1;
    name.av_val = (char*)"";  // shit
    name.av_len = 0;
  }

  if (name.av_len > 18) name.av_len = 18;

  snprintf(strRes, 255, "Name: %18.*s, ", name.av_len, name.av_val);

  if (prop->p_type == AMF_OBJECT) {
    (*pOffset) += snprintf(pStr + (*pOffset), 2048 - (*pOffset),
                           "Property: <%sOBJECT>\r\n", strRes);
    AMF_Dump_Plus(&prop->p_vu.p_object, pStr, pOffset);
    return;
  }

  else if (prop->p_type == AMF_ECMA_ARRAY) {
    (*pOffset) += snprintf(pStr + (*pOffset), 2048 - (*pOffset),
                           "Property: <%sECMA_ARRAY>\r\n", strRes);
    AMF_Dump_Plus(&prop->p_vu.p_object, pStr, pOffset);
    return;
  }

  else if (prop->p_type == AMF_STRICT_ARRAY) {
    (*pOffset) += snprintf(pStr + (*pOffset), 2048 - (*pOffset),
                           "Property: <%sSTRICT_ARRAY>\r\n", strRes);
    AMF_Dump_Plus(&prop->p_vu.p_object, pStr, pOffset);
    return;
  }

  switch (prop->p_type) {
    case AMF_NUMBER:
      snprintf(str, 255, "NUMBER:\t%.2f", prop->p_vu.p_number);
      break;

    case AMF_BOOLEAN:
      snprintf(str, 255, "BOOLEAN:\t%s",
               prop->p_vu.p_number != 0.0 ? "TRUE" : "FALSE");
      break;

    case AMF_STRING:
      snprintf(str, 255, "STRING:\t%.*s", prop->p_vu.p_aval.av_len,
               prop->p_vu.p_aval.av_val);
      break;

    case AMF_DATE:
      snprintf(str, 255, "DATE:\ttimestamp: %.2f, UTC offset: %d",
               prop->p_vu.p_number, prop->p_UTCoffset);
      break;

    default:
      snprintf(str, 255, "INVALID TYPE 0x%02x", (unsigned char)prop->p_type);
  }

  (*pOffset) += snprintf(pStr + (*pOffset), 2048 - (*pOffset),
                         "Property: <%s%s>\r\n", strRes, str);
}

void AMF_Dump_Plus(AMFObject* obj, char* pStr, int* pOffset) {
  int n;
  *pOffset +=
      snprintf(pStr + (*pOffset), 2047 - (*pOffset), "\r\n(object begin)\r\n");

  for (n = 0; n < obj->o_num; n++)
    AMFProp_Dump_Plus(&obj->o_props[n], pStr, pOffset);

  *pOffset += snprintf(pStr + (*pOffset), 2047 - (*pOffset), "(object end)");
}

Client::Client() {
  State = RTMP_STATE_INIT;
  ActiveTime = 0;
  SpsLen = 0;
  PpsLen = 0;
  EnterTime = 0;
  RecvAccTime = 0;
  HeartTime = 0;
  ReqIFrFlag = 0;
  WebrtcCLientPort = 0;
  SrcTinyId = 0;
  Seq = 0;
  ReConnectTime = 0;
  ReConnectFlag = 0;
  RecvFirstVidPkgTime = 0;
  SendFirstVidPkgTime = 0;
  HandShakeOkTime = 0;
  StartConnectTime = 0;
  ReConnectIntervalMs = 1000;
  IsSever = 0;
  AudioRtpTs = g_now_ms;
  AudioRtpTsInterVal = DEFAULT_AUDIO_RTP_TS_INTERVAL;
  IsSeiPass = 0;

  memset(&(AdtsHead), 0, sizeof(ADTS_HEAD));
}

#define RTMP_EAGAIN -2
#define RTMP_ERROR -3
#define RTMP_HNDSHK -4
#define RTMP_CHUNK -5
#define RTMP_ERR_INIT -6
#define RTMP_ERR_EXIST -7
#define RTMP_ERR_CONNECT_TIME_OUT -8

/*
static char m_RtmpState[][20] = {"INIT", "TCP_CONNECTED ", "RTMP_CONNECTED",
                                 "PLAY", "PUBLISH",        "Unknow"};

static char m_AudProfile[][20] = {"AAC_Main", "AAC_LC", "AAC_SBR",
                                  "AAC_Reserved", "Unknow"};

static char m_AudType[][20] = {
    "L_PCM",        "AD_PCM",     "MP3",        "S_PCM",       "NELLYMOSET_16",
    "NELLYMOSET_8", "NELLYMOSET", "G711 A-law", "G711 mu-law", "Opus",
    "AAC",          "SPEEX",      "UNKONW",     "UNKONW",      "MP3-8K",
    "Dev-SPEC",     "Unknow"};
    */

// static unsigned int m_sampidx[] =
//{
//    96000,
//    88200,
//    64000,
//    48000,
//    44100,
//    32000,
//    24000,
//    22050,
//    16000,
//    12000,
//    11025,
//    8000,
//    7350
//};

/*
static char m_ClientType[][20] = {"Unknow", "MiniProgram", "ThirdClient",
                                  "Unknow"};

static char TestCDN[][20] = {"Unknow", "CDN", "UPLOAD", "TRANS"};
*/

static const AVal av_NetStream_Failed =
    AVC(const_cast<char*>("NetStream.Failed"));
static const AVal av_NetStream_Play_Failed =
    AVC(const_cast<char*>("NetStream.Play.Failed"));
static const AVal av_NetStream_Play_StreamNotFound =
    AVC(const_cast<char*>("NetStream.Play.StreamNotFound"));
static const AVal av_NetConnection_Connect_InvalidApp =
    AVC(const_cast<char*>("NetConnection.Connect.InvalidApp"));
static const AVal av_NetStream_Play_Start =
    AVC(const_cast<char*>("NetStream.Play.Start"));
static const AVal av_NetStream_Play_Complete =
    AVC(const_cast<char*>("NetStream.Play.Complete"));
static const AVal av_NetStream_Play_Stop =
    AVC(const_cast<char*>("NetStream.Play.Stop"));
static const AVal av_NetStream_Seek_Notify =
    AVC(const_cast<char*>("NetStream.Seek.Notify"));
static const AVal av_NetStream_Pause_Notify =
    AVC(const_cast<char*>("NetStream.Pause.Notify"));
static const AVal av_NetStream_Play_PublishNotify =
    AVC(const_cast<char*>("NetStream.Play.PublishNotify"));
static const AVal av_NetStream_Play_UnpublishNotify =
    AVC(const_cast<char*>("NetStream.Play.UnpublishNotify"));
static const AVal av_NetStream_Publish_Start =
    AVC(const_cast<char*>("NetStream.Publish.Start"));
// static const AVal av_NetConnection_Connect_Rejected =
// AVC(const_cast<char*>("NetConnection.Connect.Rejected"));
static const AVal av_setDataFrame = AVC(const_cast<char*>("@setDataFrame"));
// #define UINT32_MAX ((unsigned int)4294967295)

int str2list(const std::string& str, const std::string& fs,
             std::vector<std::string>& arr) {
  std::vector<std::string> arrRet;
  size_t pos = 0;
  while (std::string::npos != pos) {
    size_t seqPos = str.find(fs, pos);
    if (std::string::npos == seqPos) {
      arrRet.push_back(str.substr(pos));
      break;
    } else {
      arrRet.push_back(str.substr(pos, seqPos - pos));
    }

    pos = seqPos + fs.length();
  }

  arr = arrRet;
  return arrRet.size();
}

int RtmpAssist::CalcChannel(int stream_id, int type_offset) {
  return (stream_id - 1) * 5 + type_offset;
}
int RtmpAssist::DataChannel(int stream_id) { return CalcChannel(stream_id, 4); }
int RtmpAssist::AudioChannel(int stream_id) {
  return CalcChannel(stream_id, 5);
}
int RtmpAssist::VideoChannel(int stream_id) {
  return CalcChannel(stream_id, 6);
}

int RtmpAssist::RTMP_FindPrefixProperty(AMFObject* pObj, const AVal* pName,
                                        AMFObjectProperty* pProperty) {
  int n;

  for (n = 0; n < pObj->o_num; n++) {
    AMFObjectProperty* prop = AMF_GetProp(pObj, NULL, n);

    if (prop->p_name.av_len > pName->av_len &&
        !memcmp(prop->p_name.av_val, pName->av_val, pName->av_len)) {
      memcpy(pProperty, prop, sizeof(*prop));
      return TRUE;
    }

    if (prop->p_type == AMF_OBJECT) {
      if (RTMP_FindPrefixProperty(&prop->p_vu.p_object, pName, pProperty)) {
        return TRUE;
      }
    }
  }

  return FALSE;
}

unsigned int RtmpAssist::FlvProfile2AacProfile(const FlvAacProfile aacProfile) {
  switch (aacProfile) {
    case e_FlvAacProfile_Main:
      return (unsigned int)e_AacProfile_Main;

    case e_FlvAacProfile_LC:
      return (unsigned int)e_AacProfile_LC;

    case e_FlvAacProfile_SBR:
      return (unsigned int)e_AacProfile_SSR;

    default:
      return (unsigned int)e_AacProfile_Reserved;
  }
}
RTMP_GLOBAL_INFO g_RtmpGlbobalInfo;

int RtmpAssist::WriteFile(unsigned char* pBuffer, unsigned int Len,
                          FILE* fpFlv) {
  int Ret = 0;

  if ((NULL == pBuffer) || (0 == Len) || (NULL == fpFlv)) {
    return -1;
  }

  Ret = fwrite(pBuffer, Len, 1, fpFlv);
  (void)Ret;  // to check
  return 0;
}

void RtmpAssist::WriteFileAAC(FILE* pfOutfpAAC, ADTS_HEAD* pAdtsHead,
                              const RTMPPacket* pPacket) {
  AUD_TAG_HEAD* pFlvAudTagHead = (AUD_TAG_HEAD*)pPacket->m_body;

  if (e_FlvAacPacketType_Raw == pFlvAudTagHead->AACPackType) {
    //  AAC Raw
    // | AAC Data
    // | PreviousTagSize(32)
    unsigned char* pAAcData = (unsigned char*)(pFlvAudTagHead + 1);
    int Len = pPacket->m_nBodySize - sizeof(AUD_TAG_HEAD) + sizeof(ADTS_HEAD);
    pAdtsHead->frame_lengthH = (Len >> 11) & 0x3;
    pAdtsHead->frame_lengthM = (Len >> 3) & 0xFF;
    pAdtsHead->frame_lengthL = Len & 0x7;
    WriteFile((unsigned char*)pAdtsHead, sizeof(ADTS_HEAD), pfOutfpAAC);
    Len = Len - sizeof(ADTS_HEAD);
    WriteFile(pAAcData, Len, pfOutfpAAC);
  }

  return;
}

void RtmpAssist::WriteFileH264(FILE* pfOutfpH264, const RTMPPacket* pPacket) {
  unsigned char* pBuff = (unsigned char*)pPacket->m_body;
  int FlvAvcType = pBuff[1];

  if (e_FlvAvcPacketType_Header == FlvAvcType) {
    unsigned char* pSps = NULL;
    unsigned char* pPps = NULL;
    unsigned short PpsLen = 0;
    unsigned short SpsLen = 0;
    FlvAvcHead* pFlvAvc = (FlvAvcHead*)(pBuff + 5);
    SpsLen = ntohs(*(unsigned short*)(pFlvAvc + 1));
    pSps = (unsigned char*)((unsigned char*)(pFlvAvc + 1) + 2);
    PpsLen = ntohs(*(unsigned short*)(pSps + SpsLen + 1));
    pPps = pSps + SpsLen + 1 + 2;
    unsigned char pTmpBuff[4];
    pTmpBuff[0] = 0;
    pTmpBuff[1] = 0;
    pTmpBuff[2] = 0;
    pTmpBuff[3] = 1;
    WriteFile(pTmpBuff, 4, pfOutfpH264);
    WriteFile(pSps, SpsLen, pfOutfpH264);
    WriteFile(pTmpBuff, 4, pfOutfpH264);
    WriteFile(pPps, PpsLen, pfOutfpH264);
  } else if (e_FlvAvcPacketType_Nalu == FlvAvcType) {
    if (9 >= pPacket->m_nBodySize) {
      return;
    }

    unsigned char* pNalu = pBuff + 9;
    unsigned int NaluLen = htonl(*(unsigned int*)(pBuff + 5));
    unsigned int TotalLen = 9;

    // NaluLen+9

    while (TotalLen < pPacket->m_nBodySize) {
      unsigned char pTmpBuff[4];
      pTmpBuff[0] = 0;
      pTmpBuff[1] = 0;
      pTmpBuff[2] = 0;
      pTmpBuff[3] = 1;
      WriteFile(pTmpBuff, 4, pfOutfpH264);
      WriteFile(pNalu, NaluLen, pfOutfpH264);
      TotalLen = TotalLen + NaluLen + 4;  // 加上四字节长度描述

      if (TotalLen < pPacket->m_nBodySize) {
        pNalu = pNalu + NaluLen + 4;
        NaluLen = htonl(*(unsigned int*)(pNalu - 4));
      }
    }
  }
}

void RtmpAssist::WriteFileFlv(FILE* pfOutfpFlv, const RTMPPacket* pPacket) {
  if (pPacket->m_packetType == RTMP_PACKET_TYPE_AUDIO ||
      pPacket->m_packetType == RTMP_PACKET_TYPE_VIDEO ||
      pPacket->m_packetType == RTMP_PACKET_TYPE_INFO ||
      pPacket->m_packetType == RTMP_PACKET_TYPE_FLASH_VIDEO) {
    unsigned int buflen = 131072;
    char* buf = (char*)malloc(buflen);
    unsigned int Ts = 0;

    if (NULL == buf) {
      return;
    }

    int len = WriteStream(&buf, &buflen, &Ts, pPacket);
    WriteFile((unsigned char*)buf, len, pfOutfpFlv);
    free(buf);
  }
}

int RtmpAssist::WriteStream(
    char** buf,          // target pointer, maybe preallocated
    unsigned int* plen,  // length of buffer if preallocated
    unsigned int* nTimeStamp, const RTMPPacket* pPacket) {
  unsigned int prevTagSize = 0;
  int Ret = -1, len = *plen;

  while (1) {
    char* packetBody = pPacket->m_body;
    unsigned int nPacketLen = pPacket->m_nBodySize;

    // skip video info/command packets
    if (pPacket->m_packetType == RTMP_PACKET_TYPE_VIDEO && nPacketLen == 2 &&
        ((*packetBody & 0xf0) == 0x50)) {
      Ret = 0;
      break;
    }

    if (pPacket->m_packetType == RTMP_PACKET_TYPE_VIDEO && nPacketLen <= 5) {
      tylog("ignoring too small video pPacket: size: %d", nPacketLen);
      Ret = 0;
      break;
    }

    if (pPacket->m_packetType == RTMP_PACKET_TYPE_AUDIO && nPacketLen <= 1) {
      tylog("ignoring too small audio pPacket: size: %d", nPacketLen);
      Ret = 0;
      break;
    }

#ifdef _DEBUG
    tylog("type: %02X, size: %d, TS: %d ms", pPacket->m_packetType, nPacketLen,
          pPacket->m_nTimeStamp);

    if (pPacket->m_packetType == RTMP_PACKET_TYPE_VIDEO) {
      tylog("frametype: %02X", (*packetBody & 0xf0));
    }

#endif
    // calculate pPacket size and reallocate buffer if necessary
    unsigned int Size = nPacketLen +
                        ((pPacket->m_packetType == RTMP_PACKET_TYPE_AUDIO ||
                          pPacket->m_packetType == RTMP_PACKET_TYPE_VIDEO ||
                          pPacket->m_packetType == RTMP_PACKET_TYPE_INFO)
                             ? 11
                             : 0) +
                        (pPacket->m_packetType != 0x16 ? 4 : 0);

    if ((int)Size + 4 > len) {
      /* The extra 4 is for the case of an FLV stream without a last
       * prevTagSize (we need extra 4 bytes to append it).  */
      *buf = (char*)realloc(*buf, Size + 4);

      if (*buf == 0) {
        tylog("Couldn't reallocate memory!");
        Ret = -1;  // fatal error
        break;
      }
    }

    char *ptr = *buf, *pEnd = ptr + Size + 4;

    /* audio (RTMP_PACKET_TYPE_AUDIO), video (RTMP_PACKET_TYPE_VIDEO)
     * or metadata (RTMP_PACKET_TYPE_INFO) packets: construct 11 byte
     * header then add rtmp pPacket's data.  */
    if (pPacket->m_packetType == RTMP_PACKET_TYPE_AUDIO ||
        pPacket->m_packetType == RTMP_PACKET_TYPE_VIDEO ||
        pPacket->m_packetType == RTMP_PACKET_TYPE_INFO) {
      // set data type
      //*dataType |= (((pPacket->m_packetType ==
      // RTMP_PACKET_TYPE_AUDIO)<<2)|(pPacket->m_packetType ==
      // RTMP_PACKET_TYPE_VIDEO));
      (*nTimeStamp) = pPacket->m_nTimeStamp;
      prevTagSize = 11 + nPacketLen;
      *ptr++ = pPacket->m_packetType;
      ptr = AMF_EncodeInt24(ptr, pEnd, nPacketLen);
      ptr = AMF_EncodeInt24(ptr, pEnd, *nTimeStamp);
      *ptr = (char)(((*nTimeStamp) & 0xFF000000) >> 24);
      ptr++;
      // stream id
      ptr = AMF_EncodeInt24(ptr, pEnd, 0);
    }

    memcpy(ptr, packetBody, nPacketLen);
    unsigned int len = nPacketLen;

    // correct tagSize and obtain timestamp if we have an FLV stream
    if (pPacket->m_packetType == RTMP_PACKET_TYPE_FLASH_VIDEO) {
      unsigned int pos = 0;

      while (pos + 11 < nPacketLen) {
        unsigned int dataSize = AMF_DecodeInt24(
            packetBody + pos +
            1);  // size without header (11) and without prevTagSize (4)
        *nTimeStamp = AMF_DecodeInt24(packetBody + pos + 4);
        *nTimeStamp |= (packetBody[pos + 7] << 24);
#if 0
                /* set data type */
                *dataType |= (((* (packetBody + pos) == RTMP_PACKET_TYPE_AUDIO) << 2)
                              | (* (packetBody + pos) == RTMP_PACKET_TYPE_VIDEO));
#endif

        if (pos + 11 + dataSize + 4 > nPacketLen) {
          if (pos + 11 + dataSize > nPacketLen) {
            tylog("Wrong data size (%u), stream corrupted, aborting!",
                  dataSize);
            Ret = -2;
            break;
          }

          tylog("No tagSize found, appending!");
          // we have to append a last tagSize!
          prevTagSize = dataSize + 11;
          AMF_EncodeInt32(ptr + pos + 11 + dataSize, pEnd, prevTagSize);
          Size += 4;
          len += 4;
        } else {
          prevTagSize = AMF_DecodeInt32(packetBody + pos + 11 + dataSize);
#ifdef _DEBUG
          tylog(
              "FLV Packet: type %02X, dataSize: %lu, tagSize: %lu, "
              "timeStamp: %lu ms",
              (unsigned char)packetBody[pos], dataSize, prevTagSize,
              *nTimeStamp);
#endif

          if (prevTagSize != (dataSize + 11)) {
#ifdef _DEBUG
            tylog(
                "Tag and data size are not consitent, writing tag size "
                "according to dataSize+11: %d",
                dataSize + 11);
#endif
            prevTagSize = dataSize + 11;
            AMF_EncodeInt32(ptr + pos + 11 + dataSize, pEnd, prevTagSize);
          }
        }

        pos += prevTagSize + 4;  //(11+dataSize+4);
      }
    }

    ptr += len;

    if (pPacket->m_packetType != RTMP_PACKET_TYPE_FLASH_VIDEO) {
      // FLV tag packets contain their own prevTagSize
      AMF_EncodeInt32(ptr, pEnd, prevTagSize);
      // ptr += 4;
    }

    Ret = Size;
    break;
  }

  if ((unsigned int)len > *plen) {
    *plen = len;
  }

  return Ret;  // no more media packets
}

char* RtmpPrintData(unsigned long long uiTime) {
  static char szTime[64];
  struct tm stTime;
  time_t tTime = uiTime / 1000;
  localtime_r(&tTime, &stTime);
  strftime(szTime, sizeof(szTime), "%Y_%m_%d_%H", &stTime);
  return szTime;
}

char* RtmpPrintTime(unsigned long long uiTime) {
  static char szTime[64];
  struct tm stTime;
  time_t tTime = uiTime / 1000;
  localtime_r(&tTime, &stTime);
  strftime(szTime, sizeof(szTime), "%Y_%m_%d_%H_%M_%S", &stTime);
  return szTime;
}

void DumpFileStart(Client* pClient) {
  if (1 == pClient->DumpFlag) {
    return;
  }

  assert(pClient != nullptr);

  if ((RTMP_STATE_PLAY != pClient->State) &&
      (RTMP_STATE_PUBLISH != pClient->State)) {
    tylog("pClient state=%d, start to dump file", pClient->State);

    return;
  }

  char VideoName[256] = {'\0'};
  char AudioName[256] = {'\0'};
  char FlvName[256] = {'\0'};

  char Path[256] = {'\0'};
  snprintf(Path, 255, "./RtmpDump/%s/", RtmpPrintData(g_now_ms));
  int ret = mkdir_p(Path, 0777);
  if (ret) {
    tylog("error: mkdir rtmp dir ret=%d.", ret);

    return;
  }

  snprintf(VideoName, 255, "%s%s-%llu-%llu-0x%llx-%s.264", Path,
           RtmpPrintTime(g_now_ms), pClient->SrcTinyId, pClient->TinyId,
           pClient->SessionID,
           (RTMP_STATE_PLAY == pClient->State) ? "PLAY" : "PUBLISH");
  snprintf(AudioName, 255, "%s%s-%llu-%llu-0x%llx-%s.aac", Path,
           RtmpPrintTime(g_now_ms), pClient->SrcTinyId, pClient->TinyId,
           pClient->SessionID,
           (RTMP_STATE_PLAY == pClient->State) ? "PLAY" : "PUBLISH");
  snprintf(FlvName, 255, "%s%s-%llu-%llu-0x%llx-%s.flv", Path,
           RtmpPrintTime(g_now_ms), pClient->SrcTinyId, pClient->TinyId,
           pClient->SessionID,
           (RTMP_STATE_PLAY == pClient->State) ? "PLAY" : "PUBLISH");

  DumpFileStop(pClient);

  // pClient->FlvAssistDbg.Init();
  pClient->pfOutfpH264 = fopen(VideoName, "wb");
  pClient->pfOutfpAAC = fopen(AudioName, "wb");
  pClient->pfOutfpFLV = fopen(FlvName, "wb");

  if (pClient->pfOutfpFLV) {
    Flv_header flvHeader;
    memset(&flvHeader, 0, sizeof(flvHeader));
    int Ret = 0;
    flvHeader.signature[0] = 'F';
    flvHeader.signature[1] = 'L';
    flvHeader.signature[2] = 'V';
    flvHeader.version = 1;
    flvHeader.flags = 5;  // 0000 0101
    flvHeader.offset = htonl(9);
    unsigned int tag0Size = 0;
    Ret = fwrite(&flvHeader, sizeof(flvHeader), 1, pClient->pfOutfpFLV);
    Ret = fwrite(&tag0Size, sizeof(tag0Size), 1, pClient->pfOutfpFLV);
    (void)Ret;
  }

  pClient->DumpFlag = 1;
  tylog("dumpFileStart: video:[%s] Audio:[%s] FLV[%s]", VideoName, AudioName,
        FlvName);
}

inline int GetSendLen(const char* pWorker, const char* pSendBuff) {
  if (pWorker > pSendBuff) {
    return pWorker - pSendBuff;
  } else {
    return 0;
  }
}

// 获取NALU内容的长度（由前面的四字节长度字段标识）
inline unsigned int GetNaluLen(unsigned char* pNalu) {
  return htonl(*reinterpret_cast<uint32_t*>(pNalu - 4));
}

// 将pNalu指向的NALU内容拷至pWorker，之前填充"0001"
inline int CopyNalUnit(Client* pClient, const RTMPPacket* pPkg,
                       const char* SendBuff, unsigned char* pNalu,
                       char*& pWorker, int& FrameType) {
  const unsigned int NaluLen = GetNaluLen(pNalu);

  /*先校验长度再拷贝，防止溢出*/
  int kSendLen = GetSendLen(pWorker, SendBuff) + 4 +
                 NaluLen;  // 将填充 "0001" 和 NALU 内容，

  if (VIDEO_RAW_STM_MAX_LEN <= kSendLen) {
    /*返回失败会断开连接*/

    tylog("Video Size Too Big %u Ts:%u SendLen:%u NaluLen:%u",
          pPkg->m_nBodySize, pPkg->m_nTimeStamp, kSendLen, NaluLen);
    return -2;
  }

  /*B帧识别不能通过nalutype决定*/
  if (WEB_VIDEO_FRAME_TYPE_P == FrameType) {
    WebVideoFrameType RealSliceType = GetFrameType(pNalu, NaluLen);
    if (WEB_VIDEO_FRAME_TYPE_B == RealSliceType) {
      tylog("Recv Unsupport B-frame m_nTimeStamp:%u", pPkg->m_nTimeStamp);

      pClient->RecvBframeNum++;

      return -1;
    }

    if (WEB_VIDEO_FRAME_TYPE_I == RealSliceType) {
      FrameType = RealSliceType;
    }
  }

  pWorker[0] = 0;
  pWorker[1] = 0;
  pWorker[2] = 0;
  pWorker[3] = 1;
  pWorker += 4;

  memcpy(pWorker, pNalu, NaluLen);
  pWorker += NaluLen;
  return 0;
}

void DumpFileStop(Client* pClient) {
  if (pClient->pfOutfpH264) {
    fclose(pClient->pfOutfpH264);
    pClient->pfOutfpH264 = NULL;
  }

  if (pClient->pfOutfpAAC) {
    fclose(pClient->pfOutfpAAC);
    pClient->pfOutfpAAC = NULL;
  }

  if (pClient->pfOutfpFLV) {
    fclose(pClient->pfOutfpFLV);
    pClient->pfOutfpFLV = NULL;
  }

  // pClient->FlvAssistDbg.Init();
  pClient->DumpFlag = 0;
}

static sigjmp_buf jmpenv;

static void ConnectTimeOuthandle(int) { siglongjmp(jmpenv, 1); }

int RtmpConnectTimeOut(int Timeout, RTMP* r, RTMPPacket* cp) {
  signal(SIGALRM, ConnectTimeOuthandle);

  if (sigsetjmp(jmpenv, 1) != 0) {
    tylog("RTMP connect time out");
    return RTMP_ERR_CONNECT_TIME_OUT;
  }

  struct itimerval value;
  value.it_value.tv_sec = Timeout / 1000;
  value.it_value.tv_usec = (Timeout % 1000) * 1000;
  value.it_interval.tv_sec = 0;
  value.it_interval.tv_usec = 0;
  setitimer(ITIMER_REAL, &value, NULL);

  int Ret = RTMP_Connect(r, cp);

  value.it_value.tv_sec = 0;
  value.it_value.tv_usec = 0;
  value.it_interval.tv_sec = 0;
  value.it_interval.tv_usec = 0;
  setitimer(ITIMER_REAL, &value, NULL);

  if (!Ret) return RTMP_ERR_INIT;

  return 0;
}

RtmpAssist::RtmpAssist() { RTMP_Init(&rtmp_); }

// OPT: add assignment and copy constructor; close all client
RtmpAssist::~RtmpAssist() {
  // MyRTMP_Close(&rtmp_);
  // RTMP_Free(&rtmp_);
  // CleanupClient(&m_Clients[0]);
}

static int MySendFCUnpublish(RTMP* r) {
  RTMPPacket packet;
  char pbuf[1024], *pend = pbuf + sizeof(pbuf);
  char* enc;

  packet.m_nChannel = 0x03; /* control channel (invoke) */
  packet.m_headerType = RTMP_PACKET_SIZE_MEDIUM;
  packet.m_packetType = RTMP_PACKET_TYPE_INVOKE;
  packet.m_nTimeStamp = 0;
  packet.m_nInfoField2 = 0;
  packet.m_hasAbsTimestamp = 0;
  packet.m_body = pbuf + RTMP_MAX_HEADER_SIZE;

  enc = packet.m_body;
  enc = AMF_EncodeString(enc, pend, &av_FCUnpublish);
  enc = AMF_EncodeNumber(enc, pend, ++r->m_numInvokes);
  *enc++ = AMF_NULL;
  enc = AMF_EncodeString(enc, pend, &r->Link.playpath);
  if (!enc) return FALSE;

  packet.m_nBodySize = enc - packet.m_body;

  return RTMP_SendPacket(r, &packet, FALSE);
}

static int MySendDeleteStream(RTMP* r, double dStreamId) {
  RTMPPacket packet;
  char pbuf[256], *pend = pbuf + sizeof(pbuf);
  char* enc;

  packet.m_nChannel = 0x03; /* control channel (invoke) */
  packet.m_headerType = RTMP_PACKET_SIZE_MEDIUM;
  packet.m_packetType = RTMP_PACKET_TYPE_INVOKE;
  packet.m_nTimeStamp = 0;
  packet.m_nInfoField2 = 0;
  packet.m_hasAbsTimestamp = 0;
  packet.m_body = pbuf + RTMP_MAX_HEADER_SIZE;

  enc = packet.m_body;
  enc = AMF_EncodeString(enc, pend, &av_deleteStream);
  enc = AMF_EncodeNumber(enc, pend, ++r->m_numInvokes);
  *enc++ = AMF_NULL;
  enc = AMF_EncodeNumber(enc, pend, dStreamId);

  packet.m_nBodySize = enc - packet.m_body;

  /* no response expected */
  return RTMP_SendPacket(r, &packet, FALSE);
}

static void MyAV_clear(RTMP_METHOD* vals, int num) {
  int i;
  for (i = 0; i < num; i++) free(vals[i].name.av_val);
  free(vals);
}

static void MyCloseInternal(RTMP* r, int reconnect, bool isClient) {
  int i;

  if (RTMP_IsConnected(r)) {
    if (isClient && r->m_stream_id > 0) {
      i = r->m_stream_id;
      r->m_stream_id = 0;
      if ((r->Link.protocol & RTMP_FEATURE_WRITE)) {
        MySendFCUnpublish(r);
      }
      MySendDeleteStream(r, i);
    }
    if (r->m_clientID.av_val) {
      // HTTP_Post(r, RTMPT_CLOSE, "", 1);
      free(r->m_clientID.av_val);
      r->m_clientID.av_val = NULL;
      r->m_clientID.av_len = 0;
    }
    RTMPSockBuf_Close(&r->m_sb);
  }

  r->m_stream_id = -1;
  r->m_sb.sb_socket = -1;
  r->m_nBWCheckCounter = 0;
  r->m_nBytesIn = 0;
  r->m_nBytesInSent = 0;

  if (r->m_read.flags & RTMP_READ_HEADER) {
    free(r->m_read.buf);
    r->m_read.buf = NULL;
  }
  r->m_read.dataType = 0;
  r->m_read.flags = 0;
  r->m_read.status = 0;
  r->m_read.nResumeTS = 0;
  r->m_read.nIgnoredFrameCounter = 0;
  r->m_read.nIgnoredFlvFrameCounter = 0;

  r->m_write.m_nBytesRead = 0;
  RTMPPacket_Free(&r->m_write);

  for (i = 0; i < r->m_channelsAllocatedIn; i++) {
    if (r->m_vecChannelsIn[i]) {
      RTMPPacket_Free(r->m_vecChannelsIn[i]);
      free(r->m_vecChannelsIn[i]);
      r->m_vecChannelsIn[i] = NULL;
    }
  }
  free(r->m_vecChannelsIn);
  r->m_vecChannelsIn = NULL;
  free(r->m_channelTimestamp);
  r->m_channelTimestamp = NULL;
  r->m_channelsAllocatedIn = 0;
  for (i = 0; i < r->m_channelsAllocatedOut; i++) {
    if (r->m_vecChannelsOut[i]) {
      free(r->m_vecChannelsOut[i]);
      r->m_vecChannelsOut[i] = NULL;
    }
  }
  free(r->m_vecChannelsOut);
  r->m_vecChannelsOut = NULL;
  r->m_channelsAllocatedOut = 0;
  MyAV_clear(r->m_methodCalls, r->m_numCalls);
  r->m_methodCalls = NULL;
  r->m_numCalls = 0;
  r->m_numInvokes = 0;

  r->m_bPlaying = FALSE;
  r->m_sb.sb_size = 0;

  r->m_msgCounter = 0;
  r->m_resplen = 0;
  r->m_unackd = 0;

  if (r->Link.lFlags & RTMP_LF_FTCU && !reconnect) {
    free(r->Link.tcUrl.av_val);
    r->Link.tcUrl.av_val = NULL;
    r->Link.lFlags ^= RTMP_LF_FTCU;
  }
  if (r->Link.lFlags & RTMP_LF_FAPU && !reconnect) {
    free(r->Link.app.av_val);
    r->Link.app.av_val = NULL;
    r->Link.lFlags ^= RTMP_LF_FAPU;
  }

  if (!reconnect) {
    free(r->Link.playpath0.av_val);
    r->Link.playpath0.av_val = NULL;
  }
#ifdef CRYPTO
  if (r->Link.dh) {
    MDH_free(r->Link.dh);
    r->Link.dh = NULL;
  }
  if (r->Link.rc4keyIn) {
    RC4_free(r->Link.rc4keyIn);
    r->Link.rc4keyIn = NULL;
  }
  if (r->Link.rc4keyOut) {
    RC4_free(r->Link.rc4keyOut);
    r->Link.rc4keyOut = NULL;
  }
#endif
}
void MyRTMP_Close(RTMP* r, bool isClient = false) {
  MyCloseInternal(r, 0, isClient);
}

int RtmpAssist::CleanupClient(Client* pClient) {
  MyRTMP_Close(&pClient->rtmp);

  pClient->State = RTMP_STATE_INIT;
  pClient->ActiveTime = 0;
  pClient->SpsLen = 0;
  pClient->PpsLen = 0;
  pClient->EnterTime = 0;
  pClient->RecvAccTime = 0;
  pClient->HeartTime = 0;
  pClient->ReqIFrFlag = 0;

  DumpFileStop(pClient);
  memset(&(pClient->AdtsHead), 0, sizeof(ADTS_HEAD));

  return 0;
}

int RtmpAssist::CloseStream() {
  MyRTMP_Close(&rtmp_);
  tylog("close rtmp done");

  DumpFileStop(m_Clients);

  return 0;
}

int RtmpAssist::SendConnectRes(RTMP* pRtmp, double Txn) {
  int ret = 0;

  RTMPPacket Packet;
  char pBuf[384], *pEnd = pBuf + sizeof(pBuf), *pEnc;
  AMFObject Obj;
  AMFObjectProperty p, op;
  AVal Av;
  Packet.m_nChannel = 0x03;  // control channel (invoke)
  Packet.m_headerType = RTMP_PACKET_SIZE_LARGE;
  Packet.m_packetType = RTMP_PACKET_TYPE_INVOKE;
  Packet.m_nTimeStamp = 0;
  Packet.m_nInfoField2 = 0;
  Packet.m_hasAbsTimestamp = 0;
  Packet.m_body = pBuf;
  pEnc = Packet.m_body;
  pEnc = AMF_EncodeString(pEnc, pEnd, &av__result);
  pEnc = AMF_EncodeNumber(pEnc, pEnd, Txn);
  *pEnc++ = AMF_OBJECT;
  STR2AVAL(Av, "FMS/3,5,1,525");
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_fmsVer, &Av);
  pEnc = AMF_EncodeNamedNumber(pEnc, pEnd, &av_capabilities, 31.0);
  pEnc = AMF_EncodeNamedNumber(pEnc, pEnd, &av_mode, 1.0);
  *pEnc++ = 0;
  *pEnc++ = 0;
  *pEnc++ = AMF_OBJECT_END;
  *pEnc++ = AMF_OBJECT;
  STR2AVAL(Av, "status");
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_level, &Av);
  STR2AVAL(Av, "NetConnection.Connect.Success");
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_code, &Av);
  STR2AVAL(Av, "Connection succeeded.");
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_description, &Av);
  pEnc =
      AMF_EncodeNamedNumber(pEnc, pEnd, &av_objectEncoding, pRtmp->m_fEncoding);
  STR2AVAL(p.p_name, "version");
  STR2AVAL(p.p_vu.p_aval, "3,5,1,525");
  p.p_type = AMF_STRING;
  Obj.o_num = 1;
  Obj.o_props = &p;
  op.p_type = AMF_OBJECT;
  STR2AVAL(op.p_name, "data");
  op.p_vu.p_object = Obj;
  pEnc = AMFProp_Encode(&op, pEnc, pEnd);
  *pEnc++ = 0;
  *pEnc++ = 0;
  *pEnc++ = AMF_OBJECT_END;
  Packet.m_nBodySize = pEnc - Packet.m_body;

  ret = SendChunksize(pRtmp, m_TrunkSize);
  if (ret) {
    tylog("sendChunksize ret=%d.", ret);

    return ret;
  }

  tylog("set chunk size=%d succ.", m_TrunkSize);

  bool ok = RTMP_SendPacket(pRtmp, &Packet, FALSE);
  if (!ok) {
    tylog("send fail");
    return -1;
  }

  return 0;
}

int RtmpAssist::SendCreateStreamRes(RTMP* pRtmp, double Txn, double ID) {
  RTMPPacket Packet;
  char pBuf[256];
  char* pEnd = pBuf + sizeof(pBuf);

  Packet.m_nChannel = 0x03;  // control channel (invoke)
  Packet.m_headerType = RTMP_PACKET_SIZE_MEDIUM;
  Packet.m_packetType = RTMP_PACKET_TYPE_INVOKE;
  Packet.m_nTimeStamp = 0;
  Packet.m_nInfoField2 = 0;
  Packet.m_hasAbsTimestamp = 0;
  Packet.m_body = pBuf;

  char* pEnc = Packet.m_body;
  pEnc = AMF_EncodeString(pEnc, pEnd, &av__result);
  pEnc = AMF_EncodeNumber(pEnc, pEnd, Txn);
  *pEnc++ = AMF_NULL;
  pEnc = AMF_EncodeNumber(pEnc, pEnd, ID);
  Packet.m_nBodySize = pEnc - Packet.m_body;
  pRtmp->m_stream_id = ID;

  bool ok = RTMP_SendPacket(pRtmp, &Packet, FALSE);
  if (!ok) {
    tylog("send pkt fail");
    return -1;
  }

  return 0;
}

int RtmpAssist::SendOnstatus(RTMP* pRtmp, double Txn, int streamid, int chan,
                             const char* level, const char* pCode,
                             const char* pDesc) {
  char pBuf[384];
  char *pEnd = pBuf + sizeof(pBuf), *pEnc;
  RTMPPacket Packet;
  AVal Av;

  Packet.m_nChannel = chan;
  Packet.m_headerType = RTMP_PACKET_SIZE_LARGE;
  Packet.m_packetType = RTMP_PACKET_TYPE_INVOKE;
  Packet.m_nTimeStamp = 0;
  Packet.m_nInfoField2 = streamid;
  Packet.m_hasAbsTimestamp = 0;
  Packet.m_body = pBuf;
  pEnc = Packet.m_body;
  pEnc = AMF_EncodeString(pEnc, pEnd, &av_onStatus);
  pEnc = AMF_EncodeNumber(pEnc, pEnd, Txn);
  *pEnc++ = AMF_NULL;
  *pEnc++ = AMF_OBJECT;
  STR2AVAL(Av, level);
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_level, &Av);
  STR2AVAL(Av, pCode);
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_code, &Av);
  STR2AVAL(Av, pDesc);
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_description, &Av);
  STR2AVAL(Av, "none");
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_details, &Av);
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_clientid, &Av);
  *pEnc++ = 0;
  *pEnc++ = 0;
  *pEnc++ = AMF_OBJECT_END;
  Packet.m_nBodySize = pEnc - Packet.m_body;
  bool ok = RTMP_SendPacket(pRtmp, &Packet, FALSE);
  if (!ok) {
    tylog("send pkt fail");
    return -1;
  }

  return 0;
}

int RtmpAssist::SendPlayReset(RTMP* pRtmp, const char* desc) {
  int sid = pRtmp->m_stream_id, chan = AudioChannel(sid);
  return SendOnstatus(pRtmp, 0, sid, chan, "status", "NetStream.Play.Reset",
                      desc);
}

int RtmpAssist::SendPlayStart(RTMP* pRtmp, const char* pDesc) {
  int Sid = pRtmp->m_stream_id;
  int Chan = AudioChannel(Sid);
  return SendOnstatus(pRtmp, 0, Sid, Chan, "status", "NetStream.Play.Start",
                      pDesc);
}

int RtmpAssist::SendPublishStart(RTMP* pRtmp, AMFObject* pObj,
                                 RTMPPacket* pPkg) {
  double Txn = AMFProp_GetNumber(AMF_GetProp(pObj, NULL, 1));
  int Sid = pPkg->m_nInfoField2, chan = pPkg->m_nChannel;

  return SendOnstatus(pRtmp, Txn, Sid, chan, "status",
                      "NetStream.Publish.Start", "Stream is now published");
}

int RtmpAssist::SendChunksize(RTMP* pRtmp, int Size) {
  RTMPPacket Packet;
  char pBuf[4];
  char* pEnd = pBuf + sizeof(pBuf);
  Packet.m_nChannel = 0x02;
  Packet.m_headerType = RTMP_PACKET_SIZE_MEDIUM;
  Packet.m_packetType = 0x01;
  Packet.m_body = pBuf;
  Packet.m_nBodySize = 4;

  AMF_EncodeInt32(pBuf, pEnd, Size);
  pRtmp->m_outChunkSize = Size;
  bool ok = RTMP_SendPacket(pRtmp, &Packet, FALSE);
  if (!ok) {
    tylog("send set chunk size fail");
    return -1;
  }

  return 0;
}

int RtmpAssist::SendonFCPublish(RTMP* pRtmp) {
  RTMPPacket Packet;
  char pBuf[512];
  char* pEnd = pBuf + sizeof(pBuf);

  Packet.m_nChannel = 0x03;  // control channel (invoke)
  Packet.m_headerType = RTMP_PACKET_SIZE_MEDIUM;
  Packet.m_packetType = RTMP_PACKET_TYPE_INVOKE;
  Packet.m_nTimeStamp = 0;
  Packet.m_nInfoField2 = 0;
  Packet.m_hasAbsTimestamp = 0;
  Packet.m_body = pBuf + RTMP_MAX_HEADER_SIZE;

  char* pEnc = Packet.m_body;
  pEnc = AMF_EncodeString(pEnc, pEnd, &av_onFCPublish);
  Packet.m_nBodySize = pEnc - Packet.m_body;
  bool ok = RTMP_SendPacket(pRtmp, &Packet, FALSE);
  if (!ok) {
    tylog("send pkt fail");
    return -1;
  }

  return 0;
}

int RtmpAssist::SendErr(RTMP* pRtmp, double Txn, const char* pDesc) {
  char pBuf[384];
  char* pEnd = pBuf + sizeof(pBuf);
  char* pEnc = NULL;
  ;

  RTMPPacket Packet;
  AVal av;
  Packet.m_nChannel = 0x03;  // control channel (invoke)
  Packet.m_headerType = RTMP_PACKET_SIZE_MEDIUM;
  Packet.m_packetType = RTMP_PACKET_TYPE_INVOKE;
  Packet.m_nTimeStamp = 0;
  Packet.m_nInfoField2 = 0;
  Packet.m_hasAbsTimestamp = 0;
  Packet.m_body = pBuf;

  pEnc = Packet.m_body;
  pEnc = AMF_EncodeString(pEnc, pEnd, &av__error);
  pEnc = AMF_EncodeNumber(pEnc, pEnd, Txn);
  *pEnc++ = AMF_NULL;
  *pEnc++ = AMF_OBJECT;
  STR2AVAL(av, "error");

  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_level, &av);

  STR2AVAL(av, "NetConnection.Call.Failed");
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_code, &av);

  STR2AVAL(av, pDesc);
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_description, &av);

  *pEnc++ = 0;
  *pEnc++ = 0;
  *pEnc++ = AMF_OBJECT_END;
  Packet.m_nBodySize = pEnc - Packet.m_body;
  bool ok = RTMP_SendPacket(pRtmp, &Packet, FALSE);
  if (!ok) {
    tylog("send pkt fail");
    return -1;
  }

  return 0;
}

int RtmpAssist::SendonResult(RTMP* pRtmp, double Txn) {
  RTMPPacket Packet;
  char pBuf[512];
  char* pEnd = pBuf + sizeof(pBuf);

  Packet.m_nChannel = 0x03;  // control channel (invoke)
  Packet.m_headerType = RTMP_PACKET_SIZE_MEDIUM;
  Packet.m_packetType = RTMP_PACKET_TYPE_INVOKE;
  Packet.m_nTimeStamp = 0;
  Packet.m_nInfoField2 = 0;
  Packet.m_hasAbsTimestamp = 0;
  Packet.m_body = pBuf + RTMP_MAX_HEADER_SIZE;

  char* pEnc = Packet.m_body;
  pEnc = AMF_EncodeString(pEnc, pEnd, &av__result);
  pEnc = AMF_EncodeNumber(pEnc, pEnd, Txn);
  *pEnc++ = AMF_NULL;
  Packet.m_nBodySize = pEnc - Packet.m_body;
  bool ok = RTMP_SendPacket(pRtmp, &Packet, FALSE);
  if (!ok) {
    tylog("send pkt fail");
    return -1;
  }

  return 0;
}

int RtmpAssist::SendMateData(Client*) {
  // 封装matedata发送
  RTMPPacket Packet;
  char pBuf[2048];
  char* pEnd = pBuf + sizeof(pBuf);

  Packet.m_nChannel = 0x04;  // control channel (invoke)
  Packet.m_headerType = RTMP_PACKET_SIZE_LARGE;
  Packet.m_packetType = RTMP_PACKET_TYPE_INFO;
  Packet.m_nTimeStamp = 0;
  Packet.m_nInfoField2 = 0;
  Packet.m_hasAbsTimestamp = 0;
  Packet.m_body = pBuf + RTMP_MAX_HEADER_SIZE;

  char* pEnc = Packet.m_body;
  pEnc = AMF_EncodeString(pEnc, pEnd, &av_setDataFrame);
  pEnc = AMF_EncodeString(pEnc, pEnd, &av_onMetaData);
  *pEnc++ = AMF_OBJECT;
  AVal strValue;
  strValue.av_val = (char*)"avc1";
  strValue.av_len = strlen(strValue.av_val);
  pEnc = AMF_EncodeNamedString(pEnc, pEnd, &av_videoCodecid, &strValue);
  pEnc = AMF_EncodeNamedNumber(pEnc, pEnd, &av_fileSize, 0);
  pEnc = AMF_EncodeNamedNumber(pEnc, pEnd, &av_duration, 0);
  pEnc = AMF_EncodeNamedNumber(pEnc, pEnd, &av_width, 640);
  pEnc = AMF_EncodeNamedNumber(pEnc, pEnd, &av_height, 368);
  pEnc = AMF_EncodeNamedNumber(pEnc, pEnd, &av_framerate, 30);
  *pEnc++ = 0;
  *pEnc++ = 0;
  *pEnc++ = AMF_OBJECT_END;
  Packet.m_nBodySize = pEnc - Packet.m_body;
  AMFObject Obj;
  /*AMF_Decode内部会分配内存，注意释放*/
  int Res = AMF_Decode(&Obj, Packet.m_body, Packet.m_nBodySize, FALSE);

  if (Res < 0) {
    tylog("%s, error decoding meta data packet", __FUNCTION__);
    return -1;
  }

  static char pStr[2048] = {0};
  int Len = 0;
  AMF_Dump_Plus(&Obj, pStr, &Len);
  tylog("Dercor Mate:%s", pStr);
  AMF_Reset(&Obj);
  bool ok = RTMP_SendPacket(&rtmp_, &Packet, FALSE);
  if (!ok) {
    tylog("send pkt fail");
    return -1;
  }

  return 0;
}

int RtmpAssist::ProcessPlay(RTMP* pRtmp, AMFObject* pObj, RTMPPacket*) {
  int Reset = 0;

  if (pObj->o_num > 6) {
    Reset = AMFProp_GetBoolean(AMF_GetProp(pObj, NULL, 6));
  }

  if (0 != RTMP_SendCtrl(pRtmp, 0, pRtmp->m_stream_id, 0)) {
    return RTMP_ERROR;
  }

  if (Reset) {
    if (0 != SendPlayReset(pRtmp, "Resetting stream")) {
      return RTMP_ERROR;
    }
  }

  if (0 != SendPlayStart(pRtmp, "Playing stream")) {
    return RTMP_ERROR;
  }

  return 0;
}

int RtmpAssist::ProcessPublish(RTMP* pRtmp, AMFObject* pObj, RTMPPacket* pPkg) {
  return SendPublishStart(pRtmp, pObj, pPkg);
}

int RtmpAssist::ProcessConnect(RTMP* pRtmp, AMFObject* pObj) {
  AMFObject cobj;
  AVal pname, pval;
  AMFProp_GetObject(AMF_GetProp(pObj, NULL, 2), &cobj);
  int i;

  for (i = 0; i < cobj.o_num; i++) {
    pname = cobj.o_props[i].p_name;
    pval.av_val = NULL;
    pval.av_len = 0;

    if (cobj.o_props[i].p_type == AMF_STRING) {
      pval = cobj.o_props[i].p_vu.p_aval;
    }

    if (AVMATCH(&pname, &av_objectEncoding)) {
      pRtmp->m_fEncoding = cobj.o_props[i].p_vu.p_number;
    }
  }

  return 0;
}

int RtmpAssist::HandleChunksize(RTMP* pRtmp, RTMPPacket* pPkg) {
  if (pPkg->m_nBodySize < 4) {
    tylog("%s Not enough bytes in packet", __FUNCTION__);
    return RTMP_ERROR;
  }

  pRtmp->m_inChunkSize = AMF_DecodeInt32(pPkg->m_body);
  tylog("%s Incoming chunk size changed to %d", __FUNCTION__,
        pRtmp->m_inChunkSize);
  return 0;
}

int RtmpAssist::HandleNotify(RTMP* pRtmp, RTMPPacket* pPkg) {
  AMFObject Obj;

  int Res = AMF_Decode(&Obj, pPkg->m_body, pPkg->m_nBodySize, FALSE);

  if (Res < 0) {
    tylog("%s, error decoding meta data packet", __FUNCTION__);
    return RTMP_ERROR;
  }

  AVal metastring;
  AVal metastring1;

  AMFProp_GetString(AMF_GetProp(&Obj, NULL, 0), &metastring);
  AMFProp_GetString(AMF_GetProp(&Obj, NULL, 1), &metastring1);

  if ((AVMATCH(&metastring, &av_onMetaData)) ||
      (AVMATCH(&metastring1, &av_onMetaData))) {
    AMFObjectProperty prop;

    if (RTMP_FindFirstMatchingProperty(&Obj, &av_duration, &prop)) {
      pRtmp->m_fDuration = prop.p_vu.p_number;
    }

    if (RTMP_FindPrefixProperty(&Obj, &av_video, &prop)) {
      pRtmp->m_read.dataType |= 1;
    }

    if (RTMP_FindPrefixProperty(&Obj, &av_audio, &prop)) {
      pRtmp->m_read.dataType |= 4;
    }
    if (RTMP_FindFirstMatchingProperty(&Obj, &av_videodatarate, &prop)) {
      unsigned Bitrate = (unsigned int)(prop.p_vu.p_number);
      tylog("Bitrate:%u", Bitrate);

      pRtmp->m_fDuration = prop.p_vu.p_number;
    }
  }

  AMF_Reset(&Obj);
  return 0;
}

int RtmpAssist::HandleonStatus(Client* pClient, RTMPPacket* pPkg) {
  RTMP* pRtmp = &rtmp_;
  const char* pBody = pPkg->m_body;
  unsigned int nBodySize = pPkg->m_nBodySize;
  AMFObject Obj;
  AVal Method;
  double Txn;
  int Res;

  if (pBody[0] != 0x02) /* make sure it is a string Method name we start with */
  {
    tylog("%s, Sanity failed. no string Method in invoke packet", __FUNCTION__);
    return 0;
  }

  Res = AMF_Decode(&Obj, pBody, nBodySize, FALSE);

  if (Res < 0) {
    tylog("%s, error decoding invoke packet", __FUNCTION__);
    return 0;
  }

  AMF_Dump(&Obj);
  AMFProp_GetString(AMF_GetProp(&Obj, NULL, 0), &Method);
  Txn = AMFProp_GetNumber(AMF_GetProp(&Obj, NULL, 1));
  tylog("server invoking <%s>, txn=%f.", Method.av_val, Txn);
  AMFObject Obj2;
  AVal Code, Level;
  AMFProp_GetObject(AMF_GetProp(&Obj, NULL, 3), &Obj2);
  AMFProp_GetString(AMF_GetProp(&Obj2, &av_code, -1), &Code);
  AMFProp_GetString(AMF_GetProp(&Obj2, &av_level, -1), &Level);
  tylog("%s, onStatus: %s", __FUNCTION__, Code.av_val);

  if (AVMATCH(&Code, &av_NetStream_Failed) ||
      AVMATCH(&Code, &av_NetStream_Play_Failed) ||
      AVMATCH(&Code, &av_NetStream_Play_StreamNotFound) ||
      AVMATCH(&Code, &av_NetConnection_Connect_InvalidApp)) {
    pRtmp->m_stream_id = -1;
    // MyRTMP_Close(r);
    tylog("Closing connection: %s", Code.av_val);
    return RTMP_ERROR;
  } else if (AVMATCH(&Code, &av_NetStream_Play_Start) ||
             AVMATCH(&Code, &av_NetStream_Play_PublishNotify)) {
    int i;
    pRtmp->m_bPlaying = TRUE;
    pClient->State = RTMP_STATE_PLAY;
    pClient->HandShakeOkTime = g_now_ms;

    for (i = 0; i < pRtmp->m_numCalls; i++) {
      if (AVMATCH(&pRtmp->m_methodCalls[i].name, &av_play)) {
        AV_erase(pRtmp->m_methodCalls, &pRtmp->m_numCalls, i, TRUE);
        break;
      }
    }

    tylog("%s", Code.av_val);
  } else if (AVMATCH(&Code, &av_NetStream_Publish_Start)) {
    int i;
    pRtmp->m_bPlaying = TRUE;
    pClient->State = RTMP_STATE_PUBLISH;

    for (i = 0; i < pRtmp->m_numCalls; i++) {
      if (AVMATCH(&pRtmp->m_methodCalls[i].name, &av_publish)) {
        AV_erase(pRtmp->m_methodCalls, &pRtmp->m_numCalls, i, TRUE);
        break;
      }
    }

    SendMateData(pClient);
  }
  /* Return 1 if this is a Play.Complete or Play.Stop */
  else if (AVMATCH(&Code, &av_NetStream_Play_Complete) ||
           AVMATCH(&Code, &av_NetStream_Play_Stop) ||
           AVMATCH(&Code, &av_NetStream_Play_UnpublishNotify)) {
    tylog("Play.Complete or Play.Stop: %s", Code.av_val);
    return RTMP_ERROR;
  } else if (AVMATCH(&Code, &av_NetStream_Seek_Notify)) {
    pRtmp->m_read.flags &= ~RTMP_READ_SEEKING;
  } else if (AVMATCH(&Code, &av_NetStream_Pause_Notify)) {
    if (pRtmp->m_pausing == 1 || pRtmp->m_pausing == 2) {
      RTMP_SendPause(pRtmp, FALSE, pRtmp->m_pauseStamp);
      pRtmp->m_pausing = 3;
    }
  }

  AMF_Reset(&Obj);

  return 0;
}

int HandleInvoke(RTMP* r, const char* body, unsigned int nBodySize);

int RtmpAssist::HandleMyInvoke(Client* pClient, RTMPPacket* pPkg) {
  char* pBody = pPkg->m_body;
  int Size = pPkg->m_nBodySize;
  int ret = 0;
  AMFObject Obj;
  RTMP* pRtmp = &rtmp_;

  if (RTMP_PACKET_TYPE_FLEX_MESSAGE == pPkg->m_packetType) {
    pBody++;
    Size--;
  }

  if (0x02 != *pBody) {
    tylog(
        "%s Sanity failed; no string method"
        " in invoke packet",
        __FUNCTION__);
    return RTMP_ERROR;
  }

  if (AMF_Decode(&Obj, pBody, Size, FALSE) < 0) {
    tylog("%s Error decoding invoke packet", __FUNCTION__);
    return RTMP_ERROR;
  }

  static char pStr[2048] = {0};
  int Len = 0;
  AMF_Dump_Plus(&Obj, pStr, &Len);
  tylog("recv rtmp pkt=%s", pStr);

  AVal Method;
  AMFProp_GetString(AMF_GetProp(&Obj, NULL, 0), &Method);
  tylog("recv method=%.*s.", Method.av_len, Method.av_val);

  Size = AMFProp_GetNumber(AMF_GetProp(&Obj, NULL, 1));

  if (AVMATCH(&Method, &av_connect)) {
    int ret = ProcessConnect(pRtmp, &Obj);
    if (0 != ret) {
      AMF_Reset(&Obj);
      return RTMP_ERROR;
    }

    ret = SendConnectRes(pRtmp, Size);
    if (ret) {
      tylog("send conn res ret=%d.", ret);
      return ret;
    }
  } else if (AVMATCH(&Method, &av_createStream)) {
    ret = SendCreateStreamRes(pRtmp, Size, ++m_StmId);
    if (ret) {
      tylog("sendCreateStreamRes ret=%d.", ret);

      return ret;
    }
  } else if (AVMATCH(&Method, &av_play)) {
    ret = ProcessPlay(pRtmp, &Obj, pPkg);
    if (0 == ret) {
      pClient->State = RTMP_STATE_PLAY;
      DumpFileStart(pClient);
    }
  } else if (AVMATCH(&Method, &av_publish)) {
    ret = ProcessPublish(pRtmp, &Obj, pPkg);
    if (0 == ret) {
      pClient->State = RTMP_STATE_PUBLISH;
      DumpFileStart(pClient);
    }
  } else if (AVMATCH(&Method, &av_FCPublish)) {
    ret = 0;
  } else if (AVMATCH(&Method, &av_releaseStream)) {
    ret = SendonResult(pRtmp, Size);
  } else if (AVMATCH(&Method, &av_onStatus)) {
    ret = HandleonStatus(pClient, pPkg);
  } else {
    tylog("NOTE: recv method=%.*s, not handle it.", Method.av_len,
          Method.av_val);
  }

  AMF_Reset(&Obj);
  return ret;
}

#define MAX_SPS_NUM (32)
#define MAX_PPS_NUM (256)

// todo 需要支持多pps/sps
int RtmpAssist::HandleVideoSpsAndPps(Client* pClient, const RTMPPacket* pPkg) {
  // pps和sps被单独写入AVC sequence header，需要单独处理一下
  unsigned char* pBuff = (unsigned char*)pPkg->m_body;

  if (0x1C == pBuff[0]) {
    tylog("H265 not Support!!");
    return RTMP_ERROR;
  }

  /*
     | Frametype and CodecID(8)|AVCPacketType(8)|composition time(24)|
     | cfgVersion(8) | avcProfile(8) | profileCompatibility(8) |avcLevel(8) |
     | reserved(6) | lengthSizeMinusOne(2) | reserved(3) | numOfSPS(5)
     |spsLength(16) | sps(n) | numOfPPS(8) | ppsLength(16) | pps(n) |
  */

  const unsigned char* pPos = pBuff;
  static const unsigned char NaluHeader[4] = {0, 0, 0, 1};

  pPos += 10;

  unsigned char SpsNum = (*pPos) & 0x1F;
  pPos++;
  unsigned int ReadLen = 11;

  if (MAX_SPS_NUM < SpsNum) {
    tylog("SpsNum:%u Over flow", SpsNum);
    return RTMP_ERROR;
  }

  if (1 != SpsNum) {
  }

  pClient->SpsLen = 0;

  for (int i = 0; i < SpsNum; ++i) {
    unsigned short SpsLen = pPos[0] << 8 | pPos[1];

    ReadLen = ReadLen + SpsLen + 2;

    if (pPkg->m_nBodySize <= ReadLen) {
      tylog("Read ReadLen:%u SpsLen:%u over flow", ReadLen, SpsLen);
      pClient->SpsLen = 0;
      return 0;
    }

    if (MAX_SPS_PPS_LEN <= pClient->SpsLen + SpsLen + 4) {
      tylog("SpsLen:%u over flow", pClient->SpsLen + SpsLen + 4);
      pClient->SpsLen = 0;
      return 0;
    }

    pPos += 2;

    unsigned int First4Bytes = (pPos[0] << 24) | (pPos[0 + 1] << 16) |
                               (pPos[0 + 2] << 8) | (pPos[0 + 3]);
    int SkipLen = 0;

    if (First4Bytes == 0x00000001) {
      SkipLen = 4;
    } else if ((First4Bytes & 0xFFFFFF00) == 0x00000100) {
      SkipLen = 3;
    }

    memcpy(pClient->pSps + pClient->SpsLen, NaluHeader, 4);
    pClient->SpsLen += 4;

    memcpy(pClient->pSps + pClient->SpsLen, pPos + SkipLen, SpsLen - SkipLen);
    pClient->SpsLen += SpsLen;

    H264ParseSps((char*)(pPos + SkipLen), SpsLen - SkipLen, pClient->VideoWidth,
                 pClient->VideoHeight);

    pPos += SpsLen;
  }

  pClient->PpsLen = 0;

  int PpsNum = *(pPos++);

  if (MAX_PPS_NUM < PpsNum) {
    tylog("PpsNum:%u Over flow", PpsNum);
    return RTMP_ERROR;
  }

  for (int i = 0; i < PpsNum; ++i) {
    unsigned short PpsLen = pPos[0] << 8 | pPos[1];
    ReadLen = ReadLen + PpsLen + 2;

    if (pPkg->m_nBodySize <= ReadLen) {
      tylog("Read ReadLen:%u SpsLen:%u over flow", ReadLen, PpsLen);
      pClient->PpsLen = 0;
      return 0;
    }

    if (MAX_SPS_PPS_LEN <= pClient->PpsLen + PpsLen + 4) {
      tylog("PpsLen:%u over flow", pClient->PpsLen + PpsLen + 4);
      pClient->PpsLen = 0;
      return 0;
    }

    pPos += 2;
    unsigned int First4Bytes = (pPos[0] << 24) | (pPos[0 + 1] << 16) |
                               (pPos[0 + 2] << 8) | (pPos[0 + 3]);
    int SkipLen = 0;

    if (First4Bytes == 0x00000001) {
      SkipLen = 4;
    } else if ((First4Bytes & 0xFFFFFF00) == 0x00000100) {
      SkipLen = 3;
    }

    memcpy(pClient->pPps + pClient->PpsLen, NaluHeader, 4);
    pClient->PpsLen += 4;
    memcpy(pClient->pPps + pClient->PpsLen, pPos + SkipLen, PpsLen - SkipLen);
    pClient->PpsLen += PpsLen;
    pPos += PpsLen;
  }

  pClient->SpsPpsNum++;
  pClient->IFlag = 1;
  pClient->PpsNum = PpsNum;
  pClient->SpsNum = SpsNum;

  tylog(
      "Get SpsLen:%d(include startCode 4B) PpsLen:%d(include startCode 4B) "
      "SpsNum:%u PpsNum:%u",
      pClient->SpsLen, pClient->PpsLen, SpsNum, PpsNum);
  // tylog( "pSps:%s ", VideoDumpHex((char*)pSps, SpsLen));
  // tylog( "pPps:%s",  VideoDumpHex((char*)pPps, PpsLen));
  return 0;
}

int RtmpAssist::GetFrameTypeAndPrepareSpsPps(Client* pClient,
                                             unsigned char* pNalu,
                                             char* pRawBuff,
                                             unsigned int& SendLen) {
  int NaluType = (pNalu[0] & 0x1F);

  if (kVideoNaluDelimiterRbsp == NaluType) {
    if (0 == (pNalu[1] & 0xE0)) {
      NaluType = kVideoNaluIdr;
    } else {
      NaluType = kVideoNaluSlice;
    }
  }

  if (kVideoNaluIdr == NaluType) {
    pClient->IFrameNum++;
    pClient->GopLength = g_now_ms - pClient->LastRecvIframeTime;
    pClient->LastRecvIframeTime = g_now_ms;
  }

  if (kVideoNaluIdr == NaluType) {
    if (pClient->SpsLen) {
      memcpy(pRawBuff, pClient->pSps, pClient->SpsLen);
      pRawBuff += pClient->SpsLen;
      SendLen += pClient->SpsLen;
    }

    if (pClient->PpsLen) {
      memcpy(pRawBuff, pClient->pPps, pClient->PpsLen);
      pRawBuff += pClient->PpsLen;
      SendLen += pClient->PpsLen;
    }
  }

  int FrameType = WEB_VIDEO_FRAME_TYPE_P;

  /*这里不应该有有多人编码的视频需要封装多人UDT的，不存在,私有帧类型*/
  if ((kVideoNaluSps == NaluType) || (kVideoNaluPps == NaluType) ||
      (kVideoNaluIdr == NaluType)) {
    FrameType = WEB_VIDEO_FRAME_TYPE_I;
  } else if (kVideoNaluDelimiterRbsp == NaluType) {
    if (0 == (pNalu[1] & 0xE0)) {
      FrameType = WEB_VIDEO_FRAME_TYPE_I;
    }
  }

  return FrameType;
}

// 兼容非RTMP规范, 暂不处理
int RtmpAssist::HandleVideoSliceStartCodeMod(Client* pClient,
                                             const RTMPPacket* pPkg) {
  tylog("monitor shit");
  assert(!"monitor shit");

  static unsigned char SendBuff[VIDEO_RAW_STM_MAX_LEN];

  if (9 >= pPkg->m_nBodySize) {
    return -1;
  }

  unsigned char* pBuff = (unsigned char*)pPkg->m_body;
  /*
  | Frametype and CodecID(8)|AVCPacketType(8)|composition time(24)|
  | size(32)|
  */
  /*这里需要注意内部会有多个nalu单元的情况，需要循环查找*/
  unsigned char* pNalu = pBuff + 9;
  unsigned int NaluLen = htonl(*(unsigned int*)(pBuff + 5));
  unsigned int TotalLen = 9;
  pClient->VideoFrCycle++;

  unsigned int First4Bytes = (pNalu[0] << 24) | (pNalu[0 + 1] << 16) |
                             (pNalu[0 + 2] << 8) | (pNalu[0 + 3]);
  int SkipLen = 0;

  if (First4Bytes == 0x00000001) {
    SkipLen = 4;
  } else if ((First4Bytes & 0xFFFFFF00) == 0x00000100) {
    SkipLen = 3;
  }

  pNalu += SkipLen;
  NaluLen -= SkipLen;
  char* pRawBuff = (char*)SendBuff;
  int SendLen = 0;
  int FrameType = WEB_VIDEO_FRAME_TYPE_P;
  VID_TAG_HEAD* pVideo = (VID_TAG_HEAD*)pBuff;
  FrameType = (1 == pVideo->Frametype) ? WEB_VIDEO_FRAME_TYPE_I
                                       : WEB_VIDEO_FRAME_TYPE_P;

  if (WEB_VIDEO_FRAME_TYPE_I == FrameType) {
    pClient->GopLength = g_now_ms - pClient->LastRecvIframeTime;
    pClient->LastRecvIframeTime = g_now_ms;
    pClient->IFrameNum++;

    if (pClient->SpsLen) {
      memcpy(pRawBuff, pClient->pSps, pClient->SpsLen);
      pRawBuff += pClient->SpsLen;
      SendLen += pClient->SpsLen;
    }

    if (pClient->PpsLen) {
      memcpy(pRawBuff, pClient->pPps, pClient->PpsLen);
      pRawBuff += pClient->PpsLen;
      SendLen += pClient->PpsLen;
    }
  }

  /*这里可能出现的是多slice的场景，多slice的话需要打入一个UDT的帧号，防止解码出现问题*/
  while (TotalLen < pPkg->m_nBodySize) {
    /*先校验长度再拷贝，防止溢出*/
    SendLen = SendLen + 4 + NaluLen;

    if (VIDEO_RAW_STM_MAX_LEN <= SendLen) {
      /*返回失败会断开连接*/

      tylog("Video Size Too Big %u", pPkg->m_nBodySize);
      return -2;
    }

    /*B帧识别不能通过nalutype决定*/
    if (WEB_VIDEO_FRAME_TYPE_P == FrameType) {
      WebVideoFrameType RealSliceType = GetFrameType(pNalu, NaluLen);

      if (WEB_VIDEO_FRAME_TYPE_B == RealSliceType) {
        if (0 == pClient->RecvBframeNum % 60) {
          tylog("Recv Unsupport B-frame m_nTimeStamp:%u", pPkg->m_nTimeStamp);
        }

        pClient->RecvBframeNum++;
      }
    }

    pRawBuff[0] = 0;
    pRawBuff[1] = 0;
    pRawBuff[2] = 0;
    pRawBuff[3] = 1;
    pRawBuff += 4;
    // tylog( "FrameType:%d pNalu:%s", FrameType,
    // VideoDumpHex((char*)pNalu, NaluLen));
    memcpy(pRawBuff, pNalu, NaluLen);
    pRawBuff += NaluLen;
    TotalLen += NaluLen;
    TotalLen += 4;
    TotalLen += SkipLen;

    if (TotalLen < pPkg->m_nBodySize) {
      pNalu = pNalu + NaluLen + 4;  // 加上四字节长度描述
      NaluLen = htonl(*(unsigned int*)(pNalu - 4));
      SkipLen = 0;
      First4Bytes = (pNalu[0] << 24) | (pNalu[0 + 1] << 16) |
                    (pNalu[0 + 2] << 8) | (pNalu[0 + 3]);

      if (First4Bytes == 0x00000001) {
        SkipLen = 4;
      } else if ((First4Bytes & 0xFFFFFF00) == 0x00000100) {
        SkipLen = 3;
      }

      pNalu += SkipLen;
      NaluLen -= SkipLen;
    }
  }

  // PackVideoData(SendBuff, SendLen, pPkg->m_nTimeStamp, FrameType, pClient);
  return 0;
}

int RtmpAssist::HandleVideoSliceNormal(Client* pClient,
                                       const RTMPPacket* pPkg) {
  int ret = 0;

  // 发送buffer相关
  static char SendBuff[VIDEO_RAW_STM_MAX_LEN];
  char* pWorker = SendBuff;  // pWorker指向发送buffer的空闲位置

  // pPkg->m_body 指向 Video Tag Data的FrameType字段
  const VID_TAG_HEAD* pVideo = reinterpret_cast<VID_TAG_HEAD*>(pPkg->m_body);
  int FrameType = (1 == pVideo->Frametype) ? WEB_VIDEO_FRAME_TYPE_I
                                           : WEB_VIDEO_FRAME_TYPE_P;
  bool IsPrepareSpsPps = false;  // 是否贴过 SPS PPS (I帧的第一个NALU 5前面贴)

  pClient->VideoFrCycle++;

  unsigned int NaluLen = 0;
  // TotalLen表示接收buffer中已处理的字节数
  // 加上本次NALU内容的长度和下一个NALU的四字节长度描述，移至下一个NALU内容
  for (unsigned int TotalLen = 9; TotalLen < pPkg->m_nBodySize;
       TotalLen += NaluLen + 4) {
    // pNalu指向本次待处理的NALU内容
    unsigned char* pNalu =
        reinterpret_cast<unsigned char*>(pPkg->m_body) + TotalLen;
    // 本次待处理的NALU内容的长度
    NaluLen = GetNaluLen(pNalu);
    const int NaluType = (pNalu[0] & 0x1F);

    if (!IsPrepareSpsPps) {
      unsigned int SpsPpsLen = 0;
      GetFrameTypeAndPrepareSpsPps(pClient, pNalu, pWorker, SpsPpsLen);
      if (SpsPpsLen > 0) {
        IsPrepareSpsPps = true;
        // 在GetFrameTypeAndPrepareSpsPps中贴了SpsPpsLen个字节
        pWorker += SpsPpsLen;
      }
    }

    tylog("TotalLen=%d,BodySize=%d,NaluLen=%u,NaluType=%d,FrameType=%d",
          TotalLen, pPkg->m_nBodySize, NaluLen, NaluType, FrameType);

    if (kVideoNaluSei == NaluType) {
      if (pClient->IsSeiPass) {
        int Ret =
            CopyNalUnit(pClient, pPkg, SendBuff, pNalu, pWorker, FrameType);
        if (0 != Ret) {
          return Ret;
        }

        pClient->PassSeiNum++;
      } else {
        pClient->DropSeiNum++;
      }
    } else {
      int Ret = CopyNalUnit(pClient, pPkg, SendBuff, pNalu, pWorker, FrameType);
      if (0 != Ret) {
        return Ret;
      }
    }
  }

  const int kSendLen = GetSendLen(pWorker, SendBuff);
  assert(0 != kSendLen);

  auto* pc = this->GetRtmpPeerPC();
  if (nullptr == pc) {
    return 0;
  }

  // 90kHz * 1ms
  ret = pc->rtpHandler_.DownlinkPackAndSend(
      false, {SendBuff, SendBuff + kSendLen}, 90 * pPkg->m_nTimeStamp);
  if (ret) {
    tylog("downlinkPackAndSend video ret=%d.", ret);
    return ret;
  }

  return 0;
}

int RtmpAssist::HandleVideoSlice(Client* pClient, const RTMPPacket* pPkg) {
  if (9 >= pPkg->m_nBodySize) {
    return -1;
  }

  unsigned char* pBuff = (unsigned char*)pPkg->m_body;

  // tylog( "rtmpPkg:%s",VideoDumpHex
  // ((char*)pBuff,pPkg->m_nBodySize));

  /*
  | Frametype and CodecID(8)|AVCPacketType(8)|composition time(24)|
  | size(32)|
  */
  /*这里需要注意内部会有多个nalu单元的情况，需要循环查找*/
  unsigned char* pNalu = pBuff + 9;
  unsigned int First4Bytes = (pNalu[0] << 24) | (pNalu[0 + 1] << 16) |
                             (pNalu[0 + 2] << 8) | (pNalu[0 + 3]);

  // 这里可能会有两种携带模式，一种是整包的slice通过头长度来指定，一种是通过0x0001来处理，需要区分处理
  // 携带了0x00001的模式
  if ((First4Bytes == 0x00000001) ||
      ((First4Bytes & 0xFFFFFF00) == 0x00000100)) {
    return HandleVideoSliceStartCodeMod(pClient, pPkg);
  } else {
    return HandleVideoSliceNormal(pClient, pPkg);
  }
}

int RtmpAssist::HandleVideoData(Client* pClient, const RTMPPacket* pPkg) {
  if ((NULL == pClient) || (NULL == pPkg) || (NULL == pPkg->m_body)) {
    return -10;
  }

  if (0 == pClient->RecvFirstVidPkgTime) {
    pClient->RecvFirstVidPkgTime = g_now_ms;
  }

  pClient->RecvRtmpVidNum++;
  pClient->VideoBitCycle += (pPkg->m_nBodySize << 3);
  unsigned char* pBuff = (unsigned char*)pPkg->m_body;

  if (NULL == pBuff) {
    return -13;
  }

  int FlvAvcType = pBuff[1];
  int Ret = 0;

  if (0 == pPkg->m_nTimeStamp) {
    tylog("warning? Recv Err Ts:%u FlvAvcType:%d, rtmpPkg=%s.",
          pPkg->m_nTimeStamp, FlvAvcType, RTMPPacketToString(*pPkg).data());
  }

  if (e_FlvAvcPacketType_Header == FlvAvcType) {
    Ret = HandleVideoSpsAndPps(pClient, pPkg);
  } else if (e_FlvAvcPacketType_Nalu == FlvAvcType) {
    Ret = HandleVideoSlice(pClient, pPkg);
  }

  if ((pClient->pfOutfpH264) && (0 == Ret)) {
    WriteFileH264(pClient->pfOutfpH264, pPkg);
  }

  return Ret;
}

// handle AAC head
int RtmpAssist::HandleAudioHead(Client* pClient, const RTMPPacket* pPkg) {
  AUD_TAG_HEAD* pFlvAudTagHead = (AUD_TAG_HEAD*)pPkg->m_body;
  ADTS_HEAD* pAdtsHead = &(pClient->AdtsHead);
  // AAC Sequence Header
  // | audioObjectType(5)[FlvAacProfile] |
  // samplingFrequencyIndex(4)[Mpeg2AacSample] | channelConfiguration(4) |
  // frameLengthFlag(1) | dependsOnCoreCoder(1) | extensionFlag(1) |
  AUD_AAC_SEQ_HEAD* pAAcSeqHead = (AUD_AAC_SEQ_HEAD*)(pPkg->m_body + 2);
  int SampleFrIdx = (((pAAcSeqHead->SampleFrIdxH & 0x7) << 1) |
                     (pAAcSeqHead->SampleFrIdxL & 0x1));
  tylog(
      "%s, AACPackType:%d AudFormat:0x%x SoundRate:0x%x SoundSize:0x%x "
      "SoundType:0x%x Chn:0x%x SampleFrIdx:0x%x-0x%x AudObjType:0x%x "
      "Adts:%lu",
      __FUNCTION__, pFlvAudTagHead->AACPackType,
      pFlvAudTagHead->AudFormat & 0xF, pFlvAudTagHead->SoundRate & 0x3,
      pFlvAudTagHead->SoundSize & 0x1, pFlvAudTagHead->SoundType & 0x1,
      pAAcSeqHead->ChnCfg & 0xF, SampleFrIdx,
      ((pAAcSeqHead->SampleFrIdxH & 0x7) << 1), pAAcSeqHead->AudObjType & 0x1F,
      sizeof(ADTS_HEAD));
  pClient->AudFormat = pFlvAudTagHead->AudFormat & 0xF;
  pClient->AudChn = pAAcSeqHead->ChnCfg & 0xF;

  switch (SampleFrIdx) {
    case 3:
      pClient->AudSample = 48000;
      break;

    case 4:
      pClient->AudSample = 44100;
      break;

    case 5:
      pClient->AudSample = 32000;
      break;

    case 0:
      pClient->AudSample = 96000;
      break;

    case 1:
      pClient->AudSample = 88200;
      break;

    case 6:
      pClient->AudSample = 24000;
      break;

    case 7:
      pClient->AudSample = 22050;
      break;

    case 8:
      pClient->AudSample = 16000;
      break;

    case 9:
      pClient->AudSample = 12000;
      break;

    case 10:
      pClient->AudSample = 11025;
      break;

    case 11:
      pClient->AudSample = 8000;
      break;

    case 12:
      pClient->AudSample = 7350;
      break;

    default:
      pClient->AudSample = 0;
      break;
  }

  /*还原ADTS头，用于保存AAC文件*/
  int Len = 0 + sizeof(ADTS_HEAD);
  int fullness = 0x7FF;
  pAdtsHead->syncwordH = 0xFF;
  pAdtsHead->absent = 1;
  pAdtsHead->layer = 0;
  pAdtsHead->ID = 0;
  pAdtsHead->syncwordL = 0xF;
  pAdtsHead->ChannelNumH = (pAAcSeqHead->ChnCfg >> 2) & 0x1;
  pAdtsHead->private_bit = 0;
  pAdtsHead->Sample = SampleFrIdx;

  if (e_FlvAacProfile_LC >= pAAcSeqHead->AudObjType) {
    pAdtsHead->profile =
        FlvProfile2AacProfile((FlvAacProfile)pAAcSeqHead->AudObjType);
  } else if (e_FlvAacProfile_SBR == pAAcSeqHead->AudObjType) {
    pAdtsHead->profile = 2;
  } else if (e_FlvAacProfile_HEV2 == pAAcSeqHead->AudObjType) {
    pAdtsHead->profile = 3;
  } else {
    pAdtsHead->profile = e_AacProfile_LC;
  }

  pAdtsHead->frame_lengthH = (Len >> 11) & 0x3;
  pAdtsHead->copyright_start = 0;
  pAdtsHead->copyright_bit = 0;
  pAdtsHead->home = 0;
  pAdtsHead->original_copy = 0;
  pAdtsHead->ChannelNumL = (pAAcSeqHead->ChnCfg) & 0x3;
  pAdtsHead->frame_lengthM = (Len >> 3) & 0xFF;
  pAdtsHead->fullnessH = (fullness >> 6) & 0x1F;
  pAdtsHead->frame_lengthL = Len & 0x7;
  pAdtsHead->blocks = 0;
  pAdtsHead->fullnessL = fullness & 0x3F;

  return 0;
}

int RtmpAssist::HandleAudioRawData(Client* pClient, const RTMPPacket* pPkg) {
  int ret = 0;

  auto* pc = this->GetRtmpPeerPC();
  if (nullptr == pc) {
    return 0;
  }

  //  AAC Raw
  // | AAC Data
  // | PreviousTagSize(32)
  ADTS_HEAD* pAdtsHead = &(pClient->AdtsHead);
  if (0xFF != pAdtsHead->syncwordH) {
    tylog("recv aac head syncwordH=0x%X, should be 0xFF. ret=-1",
          static_cast<uint8_t>(pAdtsHead->syncwordH));

    return -1;
  }

  pClient->LstRecvRtmpAudTs = pPkg->m_nTimeStamp;

  const char* pAacBody = pPkg->m_body + sizeof(AUD_TAG_HEAD);
  const int kAacBodyLen = pPkg->m_nBodySize - sizeof(AUD_TAG_HEAD);

  const int kAacAllLen = kAacBodyLen + sizeof(ADTS_HEAD);
  pAdtsHead->frame_lengthH = (kAacAllLen >> 11) & 0x3;
  pAdtsHead->frame_lengthM = (kAacAllLen >> 3) & 0xFF;
  pAdtsHead->frame_lengthL = kAacAllLen & 0x7;

  SrsAudioFrame f;
  f.ts_ms = pPkg->m_nTimeStamp;
  f.s.assign(reinterpret_cast<char*>(pAdtsHead), sizeof(ADTS_HEAD));
  f.s.append(pAacBody, kAacBodyLen);

  std::vector<SrsAudioFrame> outFrames;
  pc->rtpHandler_.audioTranscoderDownlink_.transcode(f, outFrames);
  if (ret) {
    tylog("audio transcode ret=%d", ret);

    return ret;
  }

  tylog("transcode aac->opus return succ, outFrames.size=%zu.",
        outFrames.size());
  // assert(outFrames.size() <= 1);  // tmp

  for (const SrsAudioFrame& outFrame : outFrames) {
    // 48kHz * 1ms
    ret = pc->rtpHandler_.DownlinkPackAndSend(
        true, {outFrame.s.begin(), outFrame.s.end()}, 48 * outFrame.ts_ms);
    if (ret) {
      tylog("downlinkPackAndSend audio ret=%d.", ret);
      return ret;
    }
  }

  return 0;
}

int RtmpAssist::HandleAudioData(Client* pClient, const RTMPPacket* pPkg) {
  int ret = 0;

  if ((NULL == pClient) || (NULL == pPkg) || (NULL == pPkg->m_body)) {
    return -11;
  }

  pClient->RecvRtmpAudNum++;
  pClient->AudioBitCycle += (pPkg->m_nBodySize << 3);

  // Audio Tag
  // | Sound Format(4)/格式AAC[0XAF] MP3 | SoundRate(2)/采样率 |
  // SoundSize(1)/位宽 | SoundType(1)/单双声道 |
  // Aac packet
  // |AACPackType(8) /AAC Sequence Header[0] AAC Raw[1]|
  AUD_TAG_HEAD* pFlvAudTagHead = (AUD_TAG_HEAD*)pPkg->m_body;
  ADTS_HEAD* pAdtsHead = &(pClient->AdtsHead);

  switch (pFlvAudTagHead->AACPackType) {
    case e_FlvAacPacketType_Header:
      ret = HandleAudioHead(pClient, pPkg);
      assert(ret == 0 && "should not use assert");
      break;

    case e_FlvAacPacketType_Raw:
      ret = HandleAudioRawData(pClient, pPkg);
      assert(ret == 0 && "should not use assert");
      break;

    default:
      tylog("unknown aac packet type=%d.", pFlvAudTagHead->AACPackType);
      return -1;
      break;
  }

  if (NULL != pClient->pfOutfpAAC) {
    WriteFileAAC(pClient->pfOutfpAAC, pAdtsHead, pPkg);
  }

  return 0;
}

int RtmpAssist::HandleClientBW(RTMP* pRtmp, const RTMPPacket* pPkg) {
  pRtmp->m_nClientBW = AMF_DecodeInt32(pPkg->m_body);

  if (pPkg->m_nBodySize > 4) {
    pRtmp->m_nClientBW2 = pPkg->m_body[4];
  } else {
    pRtmp->m_nClientBW2 = -1;
  }

  tylog("%s: client BW = %d %d", __FUNCTION__, pRtmp->m_nClientBW,
        pRtmp->m_nClientBW2);
  return 0;
}

int RtmpAssist::HandleControl(RTMP* r, RTMPPacket* pPkg) {
  int Size = pPkg->m_nBodySize, ContrlId;
  unsigned tmp = 0;

  if (Size < 6) {
    goto _CTRL_ERR_;  // includes 4-byte control id
  }

  ContrlId = AMF_DecodeInt16(pPkg->m_body);
  Size -= 2;

  switch (ContrlId) {
    case 1: {
      tmp = AMF_DecodeInt32(pPkg->m_body + 2);
      tylog("Stream EOF %u.", tmp);
      return RTMP_ERR_EXIST;
    }

    case 6: /* recv ping. reply with pong. */
      tmp = AMF_DecodeInt32(pPkg->m_body + 2);
      tylog("recv Ping %u.", tmp);
      RTMP_SendCtrl(r, 0x07, tmp, 0);
      break;

    case 7: /* recv pong. */
      tmp = AMF_DecodeInt32(pPkg->m_body + 2);
      tylog("taylor recv Pong %u, Size=%d, rtt=%d ms, get pong=%s.", tmp, Size,
            (uint32_t)(g_now_ms)-tmp, VideoDumpHex(pPkg->m_body + 2, Size));
      break;

    default:
      tylog("Unhandled control %d", ContrlId);
  }

  return 0;

_CTRL_ERR_:
  tylog("Not enough bytes in control packet");
  return RTMP_ERROR;
}

int RtmpAssist::HandleRtmpPacket(Client* pClient, RTMPPacket* pPkg) {
  RTMP* pRtmp = &rtmp_;

  if ((0 == pPkg->m_nBodySize) || (NULL == pPkg->m_body)) {
    tylog("RtmpPacket is null: %s.", RTMPPacketToString(*pPkg).data());

    return 0;
  }

  DumpFileStart(pClient);

  if (pClient->pfOutfpFLV) {
    WriteFileFlv(pClient->pfOutfpFLV, pPkg);
  }

  switch (pPkg->m_packetType) {
    case RTMP_PACKET_TYPE_CHUNK_SIZE: {
      return HandleChunksize(pRtmp, pPkg);
    }
    case RTMP_PACKET_TYPE_FLEX_MESSAGE:
    case RTMP_PACKET_TYPE_INVOKE: {
      return HandleMyInvoke(pClient, pPkg);
    }
    case RTMP_PACKET_TYPE_INFO: {
      return HandleNotify(pRtmp, pPkg);
    }
    case RTMP_PACKET_TYPE_AUDIO: {
      return HandleAudioData(pClient, pPkg);
    }
    case RTMP_PACKET_TYPE_VIDEO: {
      return HandleVideoData(pClient, pPkg);
    }
    case RTMP_PACKET_TYPE_CONTROL: {
      return HandleControl(pRtmp, pPkg);
    }
    case RTMP_PACKET_TYPE_CLIENT_BW: {
      return HandleClientBW(pRtmp, pPkg);
    }
    case RTMP_PACKET_TYPE_SERVER_BW: {
      tylog("Got server BW; not doing anything");
      break;
    }
    case RTMP_PACKET_TYPE_BYTES_READ_REPORT: {
      tylog("Got Bytes Read Report");
      break;
    }
    default: {
      tylog("NOTE: not handle packet type %d, not return err :)",
            pPkg->m_packetType);
      return 0;
    }
  }

  return 0;
}

int RtmpAssist::HandleRtmpFd(int acceptFd) {
  int ret = 0;

  Client* pClient = &this->m_Clients[g_fd2ClientIndex[acceptFd]];
  if (pClient->State < RTMP_STATE_RTMP_CONNECTED) {
    // if play, state will be PLAY
    pClient->State = RTMP_STATE_RTMP_CONNECTED;
  }
  pClient->ActiveTime = g_now_ms;

  tylog("fd2ClientIndex=%s, acceptFd=%d, rtmp sb_socket=%d.",
        tylib::AnyToString(g_fd2ClientIndex).data(), acceptFd,
        rtmp_.m_sb.sb_socket);
  assert(RTMP_IsConnected(&rtmp_));
  assert(acceptFd == rtmp_.m_sb.sb_socket);

  // OPT: should set in RTMP_Connect
  // ret = SetNonBlock(rtmp_.m_sb.sb_socket);
  // if (ret) {
  //   tylog("setNonBlock ret=%d.", ret);

  //   return ret;
  // }

  for (;;) {
    RTMPPacket rtmpPacket;
    memset(&rtmpPacket, 0, sizeof(RTMPPacket));

    do {
      bool ok = MyRTMP_ReadPacket(&rtmp_, &rtmpPacket);
      if (!ok) {
        // assert(errno != EAGAIN);
        tylog(
            "RTMP ReadPacket err errno=%d[%s] (may useless), caller should "
            "close RTMP socket.",
            errno, strerror(errno));

        return -2;
      }

      assert(rtmpPacket.m_chunk == nullptr && "we don't use chunk ptr");

      tylog("recv rtmp packet %s", RTMPPacketToString(rtmpPacket).data());
      // OPT: if not pkt come in, should break
    } while (!RTMPPacket_IsReady(&rtmpPacket));
    assert(rtmpPacket.m_body != nullptr);

    ret = HandleRtmpPacket(pClient, &rtmpPacket);
    // Always free. If not ready (m_body is null), free has no effect.
    RTMPPacket_Free(&rtmpPacket);
    if (ret) {
      tylog("handleRtmpPacket ret=%d.", ret);

      return ret;
    }
  }

  return 0;
}

// no use when use libco
int RtmpAssist::SetupListen(int) {
  int sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (-1 == sockfd) {
    fprintf(stderr, "%s, couldn't create socket", __FUNCTION__);
    tylog("couldn't create socket!\n");
    m_InitErr = 10;

    return -1;
  }

  int tmp = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &tmp, sizeof(tmp));
  /*关闭nagle算法*/
  int on = 1;
  setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
  // struct sockaddr_in addr;
  // addr.sin_family         = AF_INET;

  // if (1 == g_RtmpCfg.ListenOutIp)
  //{
  //    addr.sin_addr.s_addr    = m_OutAddr.s_addr;
  //}
  // else
  //{
  //    addr.sin_addr.s_addr    = m_InnerAddr.s_addr;
  //}

  // addr.sin_port           = htons(port);
  // int sockflags           = fcntl(sockfd, F_GETFL, 0);
  // fcntl(sockfd, F_SETFL, sockflags | O_NONBLOCK);

  // if (bind(sockfd, (struct sockaddr*) &addr, sizeof(addr)))
  //{
  //    fprintf(stderr, "%s, TCP bind failed for port number: %d\n",
  //            __FUNCTION__, port);
  //    tylog( "TCP bind failed for port number: %d errno
  //    %d(%s)\n", port, errno, strerror(errno));
  //    m_InitErr = 11;
  //    return -1;
  //}

  // if (listen(sockfd, (MAX_CLIENT << 1)) == -1)
  //{
  //    fprintf(stderr, "%s, listen failed", __FUNCTION__);
  //    tylog( "tCP listen failed port number: %d\n", port);
  //    closesocket(sockfd);
  //    m_InitErr = 12;
  //    return -1;
  //}

  // tylog( "TCP Listening on %s:%d sockfd:%d\n",
  // inet_ntoa(addr.sin_addr), port, sockfd);
  return sockfd;
}

void RtmpAssist::SetupRtmp(int acceptFd, int idx) {
  // DumpFileStop(&m_Clients[idx]);

  RTMP* pRtmp = &m_Clients[idx].rtmp;

  RTMP_Init(pRtmp);  // 只设置一些RTMP变量
  pRtmp->m_sb.sb_socket = acceptFd;

  DumpFileStop(&m_Clients[idx]);

  // opt: 写到构造函数或统一的初始化函数中

  m_Clients[idx].pfOutfpH264 = NULL;
  m_Clients[idx].pfOutfpAAC = NULL;
  m_Clients[idx].pfOutfpFLV = NULL;

  m_Clients[idx].SessionID = m_sessionidGenerator.nextid();
  m_Clients[idx].State = RTMP_STATE_TCP_CONNECTED;
  m_Clients[idx].ActiveTime = g_now_ms;
  m_Clients[idx].EnterTime = g_now_ms;
  m_Clients[idx].HeartTime = g_now_ms;
  m_Clients[idx].RecvAccTime = g_now_ms;
  m_Clients[idx].SpsLen = 0;
  m_Clients[idx].PpsLen = 0;
  m_Clients[idx].TinyId = 0;
  m_Clients[idx].IFlag = 0;
  m_Clients[idx].RecvAccVidNum = 0;
  m_Clients[idx].RecvAccAudNum = 0;
  m_Clients[idx].RecvRtmpVidNum = 0;
  m_Clients[idx].RecvRtmpAudNum = 0;
  m_Clients[idx].LstRecvAccAudTs = 0;
  m_Clients[idx].LstRecvAccVidTs = 0;
  m_Clients[idx].LstRecvRtmpAudTs = 0;
  m_Clients[idx].LstRecvRtmpVidTs = 0;
  m_Clients[idx].AudFormat = 0;
  m_Clients[idx].VideoFrCycle = 0;
  m_Clients[idx].VideoBitCycle = 0;
  m_Clients[idx].AudioBitCycle = 0;
  m_Clients[idx].IFrameNum = 0;
  m_Clients[idx].SpsPpsNum = 0;
  m_Clients[idx].VideoWidth = 0;
  m_Clients[idx].VideoHeight = 0;
  m_Clients[idx].MsgToClientNum = 0;
  m_Clients[idx].MsgToAccNum = 0;
  m_Clients[idx].ReqIFrFlag = 0;
  m_Clients[idx].ReqIFrCnt = 0;
  m_Clients[idx].LstReqIFrTimeMs = 0;
  m_Clients[idx].DropSeiNum = 0;
  m_Clients[idx].PassSeiNum = 0;
  m_Clients[idx].ClientType = RTMP_CLIENT_BUTT;
  m_Clients[idx].pfOutfpH264 = NULL;
  m_Clients[idx].pfOutfpAAC = NULL;
  m_Clients[idx].pfOutfpFLV = NULL;
  m_Clients[idx].WebrtcCLientPort = 0;
  m_Clients[idx].Seq = 0;
  m_Clients[idx].AudSeq = 0;
  m_Clients[idx].ReConnectTime = 0;
  m_Clients[idx].ReConnectFlag = 0;
  m_Clients[idx].RecvFirstVidPkgTime = 0;
  m_Clients[idx].SendFirstVidPkgTime = 0;
  m_Clients[idx].HandShakeOkTime = 0;
  m_Clients[idx].StartConnectTime = 0;
  m_Clients[idx].RecvBframeNum = 0;

  m_Clients[idx].RtmpAudPkgNumLastCycle = 0;
  m_Clients[idx].RtmpVidTsLastCycle = 0;
  m_Clients[idx].GopLength = 0;
  m_Clients[idx].LastRecvIframeTime = g_now_ms;
  m_Clients[idx].AudioRtpTs = g_now_ms;
  m_Clients[idx].AudioRtpTsInterVal = DEFAULT_AUDIO_RTP_TS_INTERVAL;
}

int MyRTMPSockBuf_Fill(RTMPSockBuf* sb) {
  int nBytes;

  if (!sb->sb_size) {
    sb->sb_start = sb->sb_buf;
  }

  while (1) {
    nBytes = sizeof(sb->sb_buf) - 1 - sb->sb_size - (sb->sb_start - sb->sb_buf);
#if defined(CRYPTO) && !defined(NO_SSL)
    if (sb->sb_ssl) {
      nBytes = TLS_read(sb->sb_ssl, sb->sb_start + sb->sb_size, nBytes);
    } else
#endif
    {
      struct pollfd pf {};
      pf.fd = sb->sb_socket;
      pf.events = (POLLIN | POLLERR | POLLHUP);
      co_poll(co_get_epoll_ct(), &pf, 1, 30000);

      assert(0 != nBytes);
      nBytes = recv(sb->sb_socket, sb->sb_start + sb->sb_size, nBytes, 0);
    }

    switch (nBytes) {
      case -1: {
        int sockerr = GetSockError();
        tylog("%s, recv returned %d. GetSockError(): %d (%s)", __FUNCTION__,
              nBytes, sockerr, strerror(sockerr));
        if (sockerr == EINTR && !RTMP_ctrlC) {
          continue;
        }

        if (sockerr == EWOULDBLOCK || sockerr == EAGAIN) {
          sb->sb_timedout = TRUE;
          nBytes = 0;

          continue;  // add by taylor
        }

        break;
      }

      case 0:
        tylog("peer close TCP conn.");
        sb->sb_timedout = FALSE;
        return 0;

        break;

      default:
        sb->sb_size += nBytes;

        break;
    }

    break;
  }

  return nBytes;
}

static int SendBytesReceived(RTMP* r) {
  RTMPPacket packet;
  char pbuf[256], *pend = pbuf + sizeof(pbuf);

  packet.m_nChannel = 0x02; /* control channel (invoke) */
  packet.m_headerType = RTMP_PACKET_SIZE_MEDIUM;
  packet.m_packetType = RTMP_PACKET_TYPE_BYTES_READ_REPORT;
  packet.m_nTimeStamp = 0;
  packet.m_nInfoField2 = 0;
  packet.m_hasAbsTimestamp = 0;
  packet.m_body = pbuf + RTMP_MAX_HEADER_SIZE;

  packet.m_nBodySize = 4;

  AMF_EncodeInt32(packet.m_body, pend, r->m_nBytesIn); /* hard coded for now */
  r->m_nBytesInSent = r->m_nBytesIn;

  /*RTMP_Log(RTMP_LOGDEBUG, "Send bytes report. 0x%x (%d bytes)", (unsigned
   * int)m_nBytesIn, m_nBytesIn); */
  return RTMP_SendPacket(r, &packet, FALSE);
}

static int MyReadN(RTMP* r, char* buffer, int n) {
  int nOriginalSize = n;
  int avail;
  char* ptr;

  r->m_sb.sb_timedout = FALSE;

#ifdef _DEBUG
  memset(buffer, 0, n);
#endif

  ptr = buffer;
  while (n > 0) {
    int nBytes = 0, nRead;
    if (r->Link.protocol & RTMP_FEATURE_HTTP) {
      // int refill = 0;
      // while (!r->m_resplen) {
      //   int ret;
      //   if (r->m_sb.sb_size < 13 || refill) {
      //     // if (!r->m_unackd) HTTP_Post(r, RTMPT_IDLE, "", 1);
      //     if (MyRTMPSockBuf_Fill(&r->m_sb) < 1) {
      //       if (!r->m_sb.sb_timedout) MyRTMP_Close(r);
      //       return 0;
      //     }
      //   }
      //   if ((ret = HTTP_read(r, 0)) == -1) {
      //     tylog("%s, No valid HTTP response found", __FUNCTION__);
      //     MyRTMP_Close(r);
      //     return 0;
      //   } else if (ret == -2) {
      //     refill = 1;
      //   } else {
      //     refill = 0;
      //   }
      // }
      // if (r->m_resplen && !r->m_sb.sb_size) MyRTMPSockBuf_Fill(&r->m_sb);
      // avail = r->m_sb.sb_size;
      // if (avail > r->m_resplen) avail = r->m_resplen;
    } else {
      avail = r->m_sb.sb_size;
      if (avail == 0) {
        int fillByte = MyRTMPSockBuf_Fill(&r->m_sb);
        if (fillByte < 1) {
          tylog("err: My RTMPSockBuf Fill=%d.", fillByte);
          if (!r->m_sb.sb_timedout) {
            // caller also close ?
            MyRTMP_Close(r);
          }

          return 0;
        }
        avail = r->m_sb.sb_size;
      }
    }
    nRead = ((n < avail) ? n : avail);
    if (nRead > 0) {
      memcpy(ptr, r->m_sb.sb_start, nRead);
      r->m_sb.sb_start += nRead;
      r->m_sb.sb_size -= nRead;
      nBytes = nRead;
      r->m_nBytesIn += nRead;
      if (r->m_bSendCounter &&
          r->m_nBytesIn > (r->m_nBytesInSent + r->m_nClientBW / 10))
        if (!SendBytesReceived(r)) return FALSE;
    }
/*tylog( "%s: %d bytes\n", __FUNCTION__, nBytes); */
#ifdef _DEBUG
    fwrite(ptr, 1, nBytes, netstackdump_read);
#endif

    if (nBytes == 0) {
      tylog("%s, RTMP socket closed by peer", __FUNCTION__);
      /*goto again; */
      MyRTMP_Close(r);
      break;
    }

    if (r->Link.protocol & RTMP_FEATURE_HTTP) r->m_resplen -= nBytes;

#ifdef CRYPTO
    if (r->Link.rc4keyIn) {
      RC4_encrypt(r->Link.rc4keyIn, nBytes, ptr);
    }
#endif

    n -= nBytes;
    ptr += nBytes;
  }

  return nOriginalSize - n;
}

static int DecodeInt32LE(const char* data) {
  unsigned char* c = (unsigned char*)data;
  unsigned int val;

  val = (c[3] << 24) | (c[2] << 16) | (c[1] << 8) | c[0];
  return val;
}

bool MyRTMP_ReadPacket(RTMP* r, RTMPPacket* packet) {
  uint8_t hbuf[RTMP_MAX_HEADER_SIZE] = {0};
  char* header = (char*)hbuf;
  int nSize, hSize, nToRead, nChunk;
  int didAlloc = FALSE;
  int extendedTimestamp;

  tylog("%s: fd=%d", __FUNCTION__, r->m_sb.sb_socket);

  if (MyReadN(r, (char*)hbuf, 1) == 0) {
    tylog("%s, failed to read RTMP packet header", __FUNCTION__);
    return FALSE;
  }

  packet->m_headerType = (hbuf[0] & 0xc0) >> 6;
  packet->m_nChannel = (hbuf[0] & 0x3f);
  header++;
  if (packet->m_nChannel == 0) {
    if (MyReadN(r, (char*)&hbuf[1], 1) != 1) {
      tylog("%s, failed to read RTMP packet header 2nd byte", __FUNCTION__);
      return FALSE;
    }
    packet->m_nChannel = hbuf[1];
    packet->m_nChannel += 64;
    header++;
  } else if (packet->m_nChannel == 1) {
    int tmp;
    if (MyReadN(r, (char*)&hbuf[1], 2) != 2) {
      tylog("%s, failed to read RTMP packet header 3nd byte", __FUNCTION__);
      return FALSE;
    }
    tmp = (hbuf[2] << 8) + hbuf[1];
    packet->m_nChannel = tmp + 64;
    tylog("%s, m_nChannel: %0x", __FUNCTION__, packet->m_nChannel);
    header += 2;
  }

  nSize = packetSize[packet->m_headerType];

  if (packet->m_nChannel >= r->m_channelsAllocatedIn) {
    int n = packet->m_nChannel + 10;
    int* timestamp =
        static_cast<int*>(realloc(r->m_channelTimestamp, sizeof(int) * n));
    RTMPPacket** packets = static_cast<RTMPPacket**>(
        realloc(r->m_vecChannelsIn, sizeof(RTMPPacket*) * n));
    if (!timestamp) free(r->m_channelTimestamp);
    if (!packets) free(r->m_vecChannelsIn);
    r->m_channelTimestamp = timestamp;
    r->m_vecChannelsIn = packets;
    if (!timestamp || !packets) {
      r->m_channelsAllocatedIn = 0;
      return FALSE;
    }
    memset(r->m_channelTimestamp + r->m_channelsAllocatedIn, 0,
           sizeof(int) * (n - r->m_channelsAllocatedIn));
    memset(r->m_vecChannelsIn + r->m_channelsAllocatedIn, 0,
           sizeof(RTMPPacket*) * (n - r->m_channelsAllocatedIn));
    r->m_channelsAllocatedIn = n;
  }

#define RTMP_LARGE_HEADER_SIZE 12
  if (nSize ==
      RTMP_LARGE_HEADER_SIZE) /* if we get a full header the timestamp is
                                 absolute */
    packet->m_hasAbsTimestamp = TRUE;

  else if (nSize < RTMP_LARGE_HEADER_SIZE) { /* using values from the last
                                                message of this channel */
    if (r->m_vecChannelsIn[packet->m_nChannel])
      memcpy(packet, r->m_vecChannelsIn[packet->m_nChannel],
             sizeof(RTMPPacket));
  }

  nSize--;

  if (nSize > 0 && MyReadN(r, header, nSize) != nSize) {
    tylog("%s, failed to read RTMP packet header. type: %x", __FUNCTION__,
          (unsigned int)hbuf[0]);
    return FALSE;
  }

  hSize = nSize + (header - (char*)hbuf);

  if (nSize >= 3) {
    packet->m_nTimeStamp = AMF_DecodeInt24(header);

    /*tylog( "%s, reading RTMP packet chunk on channel %x,
     * headersz %i, timestamp %i, abs timestamp %i", __FUNCTION__,
     * packet.m_nChannel, nSize, packet.m_nTimeStamp, packet.m_hasAbsTimestamp);
     */

    if (nSize >= 6) {
      packet->m_nBodySize = AMF_DecodeInt24(header + 3);
      packet->m_nBytesRead = 0;

      if (nSize > 6) {
        packet->m_packetType = header[6];

        if (nSize == 11) packet->m_nInfoField2 = DecodeInt32LE(header + 7);
      }
    }
  }

  extendedTimestamp = packet->m_nTimeStamp == 0xffffff;
  if (extendedTimestamp) {
    if (MyReadN(r, header + nSize, 4) != 4) {
      tylog("%s, failed to read extended timestamp", __FUNCTION__);
      return FALSE;
    }
    packet->m_nTimeStamp = AMF_DecodeInt32(header + nSize);
    hSize += 4;
  }

  RTMP_LogHexString(RTMP_LOGDEBUG2, (uint8_t*)hbuf, hSize);

  if (packet->m_nBodySize > 0 && packet->m_body == NULL) {
    if (!RTMPPacket_Alloc(packet, packet->m_nBodySize)) {
      tylog("%s, failed to allocate packet", __FUNCTION__);
      return FALSE;
    }
    didAlloc = TRUE;
    packet->m_headerType = (hbuf[0] & 0xc0) >> 6;
  }

  nToRead = packet->m_nBodySize - packet->m_nBytesRead;
  nChunk = r->m_inChunkSize;
  if (nToRead < nChunk) {
    nChunk = nToRead;
  }

  /* Does the caller want the raw chunk? */
  if (packet->m_chunk) {
    packet->m_chunk->c_headerSize = hSize;
    memcpy(packet->m_chunk->c_header, hbuf, hSize);
    packet->m_chunk->c_chunk = packet->m_body + packet->m_nBytesRead;
    packet->m_chunk->c_chunkSize = nChunk;
  }

  int readByteNum = MyReadN(r, packet->m_body + packet->m_nBytesRead, nChunk);
  if (readByteNum != nChunk) {
    tylog(
        "%s, failed to read RTMP packet body. len: %u, readByte=%d, chunk=%d.",
        __FUNCTION__, packet->m_nBodySize, readByteNum, nChunk);
    return FALSE;
  }

  RTMP_LogHexString(RTMP_LOGDEBUG2,
                    (uint8_t*)packet->m_body + packet->m_nBytesRead, nChunk);

  packet->m_nBytesRead += nChunk;

  /* keep the packet as ref for other packets on this channel */
  if (!r->m_vecChannelsIn[packet->m_nChannel])
    r->m_vecChannelsIn[packet->m_nChannel] =
        static_cast<RTMPPacket*>(malloc(sizeof(RTMPPacket)));
  memcpy(r->m_vecChannelsIn[packet->m_nChannel], packet, sizeof(RTMPPacket));
  if (extendedTimestamp) {
    r->m_vecChannelsIn[packet->m_nChannel]->m_nTimeStamp = 0xffffff;
  }

  if (RTMPPacket_IsReady(packet)) {
    /* make packet's timestamp absolute */
    if (!packet->m_hasAbsTimestamp)
      packet->m_nTimeStamp +=
          r->m_channelTimestamp[packet->m_nChannel]; /* timestamps seem to be
                                                        always relative!! */

    r->m_channelTimestamp[packet->m_nChannel] = packet->m_nTimeStamp;

    /* reset the data from the stored packet. we keep the header since we may
     * use it later if a new packet for this channel */
    /* arrives and requests to re-use some info (small packet header) */
    r->m_vecChannelsIn[packet->m_nChannel]->m_body = NULL;
    r->m_vecChannelsIn[packet->m_nChannel]->m_nBytesRead = 0;
    r->m_vecChannelsIn[packet->m_nChannel]->m_hasAbsTimestamp =
        FALSE; /* can only be false if we reuse header */
  } else {
    packet->m_body = NULL; /* so it won't be erased on free */
  }

  return TRUE;
}

}  // namespace tywebrtc