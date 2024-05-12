// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtmp/FlvAssist.h"

#include <cassert>
#include <climits>
#include <cstdio>
#include <cstring>

#include "src/global_tmp/h264SpsDec.h"
#include "src/log/log.h"
#include "src/rtmp/CommonAssist.h"
#include "src/rtmp/rtmp_handler.h"

namespace tywebrtc {

FlvAssist::FlvAssist(RtmpHandler& belongingRtmpHandler)
    : belongingRtmpHandler_(belongingRtmpHandler) {
  mStartTimeAud = 0;
  mStartTimeVid = 0;
  mVideoConfigTag = false;
  mAudioConfigTag = false;
  m_LastSampleRate = 0;
  mAudioFramecnt = 0;
}

// 0x00 00 00 01：NALU对应的slice为一帧的开始
// 0x00 00 01：   NALU对应的slice非一帧的开始
// 但是，在单核编码情况下，一帧只编一个slice，所以开始码可能是0x00 00 01
const unsigned char FlvAssist::sFrameStartCode[] = {0x00, 0x00, 0x00, 0x01};
const unsigned char FlvAssist::sSliceStartCode[] = {0x00, 0x00, 0x01};

// static bool VideoConfigTag = false;
// static bool AudioConfigTag = false;
// static unsigned int g_startTime =0;
// static unsigned int g_startTimeV =0;

// send a video frame
int FlvAssist::SendVideoFrame(const std::vector<char>& h264Frame,
                              uint64_t frameMs) {
  int ret = 0;

  const char* pRawDataBuff = h264Frame.data();
  int RawBuffLen = h264Frame.size();

  int NalHeadPos = 0;

  while (0 == pRawDataBuff[NalHeadPos]) {
    ++NalHeadPos;
  }

  NalHeadPos++;
  enVideoH264NaluType NaluType =
      static_cast<enVideoH264NaluType>(pRawDataBuff[NalHeadPos] & 0x1F);

  if (kVideoNaluDelimiterRbsp == NaluType) {
    if (0 == (pRawDataBuff[NalHeadPos + 1] & 0xE0)) {
      NaluType = kVideoNaluIdr;
    }
  }

  /*没有收到I帧立即请求I帧，并设置请求I帧标志位置，后续由定时器触发请求*/

  MediaBuffer mediaBuffer;
  mediaBuffer.setMediaTime(frameMs);
#define VIDEO_RAW_STM_MAX_LEN (2048000)
#define AUDIO_RAW_STM_MAX_LEN (4096)
#define UDP_RECV_BUFF_MAX_LEN (16384)
  int dstBufLen = VIDEO_RAW_STM_MAX_LEN;
  static uint8_t dstBuf[VIDEO_RAW_STM_MAX_LEN];
  mediaBuffer.init(pRawDataBuff, RawBuffLen, dstBuf, dstBufLen);
  mediaBuffer.setCompositionTimeOffset(0);
  mediaBuffer.setFrameType(NaluType);
  ret = makeTag(mediaBuffer, e_MediaType_Avc);
  if (ret) {
    tylog("make tag ret=%d.", ret);

    return ret;
  }

  if (mContext.mSpsLength > 0) {
    int width = 0;
    int height = 0;
    H264ParseSps((char*)(mContext.mSps), mContext.mSpsLength, width, height);
  }

  assert(this->belongingRtmpHandler_.mRtmpInstance != nullptr);

  // pClient->VideoBitCycle += (mediaBuffer.getDestContentLength() << 3);
  int bufRet = RTMP_Write(this->belongingRtmpHandler_.mRtmpInstance,
                          (char*)mediaBuffer.getDestBuffer(),
                          mediaBuffer.getDestContentLength());
  if (bufRet <= 0) {
    tylog("rtmp write return value=%d, error", bufRet);
    return -1;
  }
  tylog("rtmp write buf len=%d, success", bufRet);

  /*
    if (pClient->pfOutfpFLV) {
      MediaBuffer mediaBufferDbg;
      mediaBufferDbg.setMediaTime(frameMs);
      mediaBufferDbg.init(pRawDataBuff, RawBuffLen, (unsigned char*)dstBuf,
                          dstBufLen);
      mediaBufferDbg.setCompositionTimeOffset(0);
      mediaBufferDbg.setFrameType((enVideoH264NaluType)NaluType);
      pClient->FlvAssistDbg.makeTag(mediaBufferDbg, e_MediaType_Avc);
      WriteFile(mediaBufferDbg.getDestBuffer(),
                mediaBufferDbg.getDestContentLength(), pClient->pfOutfpFLV);
    }

    if (pClient->pfOutfpH264) {
      WriteFile(pRawDataBuff, RawBuffLen, pClient->pfOutfpH264);
    }
  */

  return 0;
}

int FlvAssist::SendAudioFrame(const std::vector<char>& audioFrame,
                              uint64_t frameMs) {
  int ret = 0;

  const char* pRawDataBuff = audioFrame.data();
  // memcpy(&pClient->AdtsHead, pRawDataBuff, sizeof (ADTS_HEAD));

  int RawBuffLen = audioFrame.size();

  // pClient->AudioBitCycle += (RawBuffLen << 3);
  MediaBuffer mediaBuffer;
  mediaBuffer.setMediaTime(frameMs);

  static unsigned char dstBuf[AUDIO_RAW_STM_MAX_LEN];
  int dstBufLen = AUDIO_RAW_STM_MAX_LEN;
  mediaBuffer.init((unsigned char*)pRawDataBuff, RawBuffLen,
                   (unsigned char*)dstBuf, dstBufLen);

  ret = makeTag(mediaBuffer, e_MediaType_Aac);
  if (ret) {
    tylog("make tag ret=%d.", ret);

    return ret;
  }

  assert(this->belongingRtmpHandler_.mRtmpInstance != nullptr);

  int bufRet = RTMP_Write(this->belongingRtmpHandler_.mRtmpInstance,
                          (char*)mediaBuffer.getDestBuffer(),
                          mediaBuffer.getDestContentLength());
  if (0 >= bufRet) {
    tylog("rtmp write return value=%d, error", bufRet);
    return -1;
  }

  //     if (pClient->pfOutfpAAC)
  //         WriteFile (pRawDataBuff, RawBuffLen, pClient->pfOutfpAAC);
  //
  //     if (pClient->pfOutfpFLV)
  //     {
  //         MediaBuffer mediaBuffer;
  //         mediaBuffer.setMediaTime (pAudio->TimeStamp);
  //         mediaBuffer.init ( (unsigned char*) pRawDataBuff, RawBuffLen,
  //         (unsigned char*) dstBuf,
  //                            dstBufLen);
  //         pClient->FlvAssistDbg.makeTag (mediaBuffer, e_MediaType_Aac);
  //         WriteFile (mediaBuffer.getDestBuffer(),
  //         mediaBuffer.getDestContentLength(), pClient->pfOutfpFLV);
  //     }

  return 0;
}

int FlvAssist::makeTag(MediaBuffer& mediaBuffer, MediaType mediaType) {
  int ret = 0;
  ret = mediaBuffer.checkBuf();
  if (0 != ret) {
    tylog("mediaBuffer check buf ret=%d", ret);

    return ret;
  }

  switch (mediaType) {
    case e_MediaType_Avc:
      ret = makeAvcTag(mediaBuffer);
      if (ret) {
        tylog("make avc tag ret=%d", ret);

        return ret;
      }
      break;

    case e_MediaType_Aac:
      ret = makeAacTag(mediaBuffer);
      if (ret) {
        tylog("make aac tag ret=%d", ret);

        return ret;
      }
      break;

    default:
      assert(!"unknown make tag type");
      ret = -3;
      break;
  }

  return ret;
}

void FlvAssist::Init() {
  mStartTimeAud = 0;
  mStartTimeVid = 0;
  mVideoConfigTag = false;
  mAudioConfigTag = false;
  m_LastSampleRate = 0;
  mAudioFramecnt = 0;

  mContext.mFrameType = kVideoNaluUnspecific;
  mContext.mSpsLength = 0;
  mContext.mPpsLength = 0;
  memset(mContext.mSps, 0, sizeof(mContext.mSps));
  memset(mContext.mPps, 0, sizeof(mContext.mPps));
}

int FlvAssist::CheckNalu(unsigned char* buffer, unsigned int buffer_len) {
  int buffer_rest_len = buffer_len;
  unsigned char* avc_buffer_start_pos = buffer;

  unsigned int nalu_len = ntohl(*((unsigned int*)avc_buffer_start_pos));

  while (buffer_rest_len > 0) {
    buffer_rest_len = buffer_rest_len - (4 + nalu_len);
    avc_buffer_start_pos = avc_buffer_start_pos + (4 + nalu_len);

    nalu_len = ntohl(*((unsigned int*)avc_buffer_start_pos));
  }

  return 0;
}

char* VideoDumpHex(char* data, int len) {
  static char buf[65535];
  int size = 0;
  int i = 0;
  size += sprintf(buf + size, "\nData len %d\n", len);
  for (i = 0; i < len; ++i) {
    size += sprintf(buf + size, "%02X ", (unsigned char)data[i]);
    if ((i % 4) == 3) size += sprintf(buf + size, "| ");
    if ((i % 16) == 15) size += sprintf(buf + size, "\n");
  }
  return buf;
}

int FlvAssist::makeAvcTag(MediaBuffer& mediaBuffer) {
  // FLV Tag
  // | TagType(8) | DataSize(24) | Timestamp(24) | TimestampExtended(8) |
  // StreamID(24) |
  // Video Tag
  // | FrameType(4) | CodecID(4) | AVCPackType(8) | CompositionTime(24) |
  // Avc Packet
  // | AVC Nalu Lenght(32) | AVC Nalu ES |
  // | PreviousTagSize(32)
  /*VIDEO_TAG*/
  int ret = 0;
  int dstBufLen = DST_BUF_LEN;
  static unsigned char AVcdstBuf[DST_BUF_LEN];  // 2M is safe?
  MediaBuffer AvcBuffer;
  AvcBuffer.init(mediaBuffer.getSrcBuffer(), mediaBuffer.getSrcBufferLength(),
                 (unsigned char*)AVcdstBuf, dstBufLen);

  AvcBuffer.setFrameType(mediaBuffer.getFrameType());
  ret = toAvc(AvcBuffer);

  if (0 != ret) return -1;

  int dataSizePos = 0;
  int configTagSize = 0;
  unsigned int timeStamp = mediaBuffer.getMediaTime();
  CommonAssist commonAssist;
  commonAssist.setupBuffer(mediaBuffer.getDestBuffer(),
                           mediaBuffer.getDestBufferLength());

  if ((kVideoNaluSps == mediaBuffer.getFrameType()) ||
      (kVideoNaluPps == mediaBuffer.getFrameType()) ||
      (kVideoNaluIdr == mediaBuffer.getFrameType())) {
    if (mContext.mSpsLength > 0 && mContext.mPpsLength > 0) {
      if (0 == mStartTimeVid) mStartTimeVid = timeStamp;

      unsigned long long u64TimeOffset = 0;
      u64TimeOffset = timeStamp;
      commonAssist.putByte(e_FlvTagType_Video);  // Tag Type
      dataSizePos = commonAssist.getContentLength();
      commonAssist.putBit24(0);                            // DataSize(24)
      commonAssist.putBit24((unsigned int)u64TimeOffset);  // Timestamp(24)
      commonAssist.putByte(((unsigned int)u64TimeOffset) >>
                           24);  // TimestampExtended(8)
      commonAssist.putBit24(0);  // StreamID(24), always 0
      commonAssist.putByte(e_FlvVideoFrameType_Key |
                           e_FlvVideoCodecId_Avc);      // Frametype and CodecID
      commonAssist.putByte(e_FlvAvcPacketType_Header);  // AVCPacketType
      commonAssist.putBit24(0);                         // composition time
      /*
      AVCDecorderConfigurationRecord包括文件的信息。
      具体格式如下：
      | cfgVersion(8) | avcProfile(8) | profileCompatibility(8) |avcLevel(8) |
      | reserved(6) | lengthSizeMinusOne(2) | reserved(3) | numOfSPS(5)
      |spsLength(16) | sps(n) | numOfPPS(8) | ppsLength(16) | pps(n) |
      */
      commonAssist.putByte(1);                 // version
      commonAssist.putByte(mContext.mSps[1]);  // profile
      commonAssist.putByte(mContext.mSps[2]);  // profile
      commonAssist.putByte(mContext.mSps[3]);  // level
      commonAssist.putByte(
          0xff);  // 6 bits reserved (111111) + 2 bits nal size length - 1 (11)
      commonAssist.putByte(
          0xe1);  // 3 bits reserved (111) + 5 bits number of sps (00001)
      commonAssist.putBit16(mContext.mSpsLength);                 // sps length
      commonAssist.putBytes(mContext.mSps, mContext.mSpsLength);  // sps data
      commonAssist.putByte(1);                     // number of pps
      commonAssist.putBit16(mContext.mPpsLength);  // pps length
      commonAssist.putBytes(mContext.mPps, mContext.mPpsLength);  // sps data
      commonAssist.putBit24At(commonAssist.getContentLength() - 11,
                              dataSizePos);  // rewrite Data size
      commonAssist.putBit32(commonAssist.getContentLength());  // Pre tag size
      configTagSize = commonAssist.getContentLength();
      mVideoConfigTag = true;
    } else {
      return 0;  // 忽略该数据（可能是先来的P帧、重复的sps等）
    }
  }

  // unsigned char*      avc_buffer_start_pos    = AvcBuffer.getDestBuffer();
  // unsigned int        nalu_len                = ntohl (* ( (unsigned int*)
  // avc_buffer_start_pos));
  // int                 buffer_rest_len         =
  // AvcBuffer.getDestContentLength();
  unsigned long long u64TimeOffset = 0;

  u64TimeOffset = timeStamp;

  commonAssist.putByte(e_FlvTagType_Video);  // Tag Type
  dataSizePos = commonAssist.getContentLength();
  commonAssist.putBit24(0);  // DataSize(24)

  commonAssist.putBit24((unsigned int)u64TimeOffset);  // Timestamp(24)
  commonAssist.putByte(((unsigned int)u64TimeOffset) >>
                       24);  // TimestampExtended(8)
  commonAssist.putBit24(0);  // StreamID(24), always 0

  commonAssist.putByte(
      AvcFrameType2FlvVideoFrameType(mediaBuffer.getFrameType()) |
      e_FlvVideoCodecId_Avc);                     // Frametype and CodecID
  commonAssist.putByte(e_FlvAvcPacketType_Nalu);  // AVCPacketType
  commonAssist.putBit24(
      mediaBuffer.getCompositionTimeOffset());  // CompositionTime
  commonAssist.putBytes(AvcBuffer.getDestBuffer(),
                        AvcBuffer.getDestContentLength());  // Data
  commonAssist.putBit24At(commonAssist.getContentLength() - 11 - configTagSize,
                          dataSizePos);  // rewrite Data size
  commonAssist.putBit32(commonAssist.getContentLength() -
                        configTagSize);  // Pre tag size

  // buffer_rest_len = buffer_rest_len - (4 + nalu_len);
  // avc_buffer_start_pos = avc_buffer_start_pos + (4 + nalu_len);
  // nalu_len = ntohl (* ( (unsigned int*) avc_buffer_start_pos));

  mediaBuffer.setDestContentLength(commonAssist.getContentLength());
  return ret;
}

int FlvAssist::makeAacTag(MediaBuffer& mediaBuffer) {
  // FLV Tag
  // | TagType(8) | DataSize(24) | Timestamp(24) | TimestampExtended(8) |
  // StreamID(24) |
  // Audio Tag
  // | Sound Format(4) | SoundRate(2) | SoundSize(1) | SoundType(1) |
  // Aac packet
  // AACPackType(8) |
  // AAC Sequence Header or AAC Raw
  //     AAC Sequence Header
  // | audioObjectType(5) | samplingFrequencyIndex(4) | channelConfiguration(4)
  // | frameLengthFlag(1) | dependsOnCoreCoder(1) | extensionFlag(1) |
  //     AAC Raw
  // | AAC Data
  // | PreviousTagSize(32)

  // https://wiki.multimedia.cx/index.php/ADTS
  //首先判断是否带ADTS头
  const uint8_t* srcBuffer =
      static_cast<const uint8_t*>(mediaBuffer.getSrcBuffer());
  if (!(srcBuffer[0] == 0xFF && (srcBuffer[1] & 0xF6) == 0xF0))  // from faad
  {
    tylog("have no ADTS head, srcBuffer[0]=0x%X, [1]=0x%X.", srcBuffer[0],
          srcBuffer[1]);

    return -1;
  }

  int dataSizePos = 0;
  int configTagSize = 0;
  unsigned int timeStamp = mediaBuffer.getMediaTime();
  // tylog( "debug ts=%u", timeStamp);
  CommonAssist commonAssist;
  commonAssist.setupBuffer(mediaBuffer.getDestBuffer(),
                           mediaBuffer.getDestBufferLength());

  unsigned char profile = ((srcBuffer[2] & 0xc0) >> 6);
  unsigned char sample_rate = (srcBuffer[2] & 0x3c) >> 2;
  unsigned char channel =
      ((srcBuffer[2] & 0x1) << 2) | ((srcBuffer[3] & 0xc0) >> 6);

  if (m_LastSampleRate != sample_rate) {
    mAudioConfigTag = false;
  }

  //每隔200个音频发一个tag
  if (mAudioFramecnt++ % 200 == 0) {
    mAudioConfigTag = false;
  }

  m_LastSampleRate = sample_rate;

  /*AUDIO_TAG*/
  if (!mAudioConfigTag) {
    if (0 == mStartTimeAud) {
      mStartTimeAud = timeStamp;
    }

    unsigned long long u64TimeOffset = 0;

    u64TimeOffset = timeStamp;
    commonAssist.putByte(e_FlvTagType_Audio);  // Tag Type
    dataSizePos = commonAssist.getContentLength();
    commonAssist.putBit24(0);  // DataSize(24)
    // commonAssist.putBit24(timeStamp);         // Timestamp(24)
    // commonAssist.putByte(timeStamp >> 24);    // TimestampExtended(8)
    commonAssist.putBit24((unsigned int)u64TimeOffset);  // Timestamp(24)
    commonAssist.putByte(((unsigned int)u64TimeOffset) >>
                         24);  // TimestampExtended(8)
    commonAssist.putBit24(0);  // StreamID(24)

    /*Audio Tag Head*/
    /*
    SoundFormat UB[4] Format of SoundData
            10 = AAC
    SoundRate UB[2] Sampling rate
            0 = 5.5-kHz
            1 = 11-kHz
            2 = 22-kHz
            3 = 44-kHz
            For AAC: always 3
    SoundSize UB[1] Size of each sample
            0 = snd8Bit
            1 = snd16Bit
    SoundType UB[1] Mono or stereo sound
            0 = sndMono
            1 = sndStereo
    For AAC: always 1
    */
    commonAssist.putByte(0xAF);  // For AAC: always 0xAF
    commonAssist.putByte(
        0);  // AAC Packet Type: 0 = AAC sequence header; 1 = AAC raw
    // if(AAC Packet Type == 0) AudioSpecificConfig
    /*
    AudioSpecificConfig靠靠縄SO
    14496-3縣ttp://www.nhzjj.com/asp/admin/editor/newsfile/2010318163752818.pdf?
            audioObjectType:         5 bits?緼AC-LC?縎BR?9縋S
            samplingFrequencyIndex:  4 bits靠靠靠靠靠
            channelConfiguration:    4 bits靠靠
            if (audioObjectType == 5 || audioObjectType == 29)
                    extensionSamplingFrequencyIndex: 4 bits靠靠靠靠?
                    audioObjectType:                 5 bits靠靠靠緼OT
            GASpecificConfig
            frameLengthFlag:         1 bit?靠靠1024?靠靠960
            DependsOnCoreCoder:      1 bit
            extensionFlag:           1 bit

    AudioSpecificConfig靠靠ADTS?靠靠靠靠3靠靠?
            byte profile(LC,Main,HE) = ((payload[2]&0xc0)>>6)+1;
            byte sample_rate         = (payload[2]&0x3c)>>2;
            byte channel             =
    ((payload[2]&0x1)<<2)|((payload[3]&0xc0)>>6);
    靠靠3靠靠靠2靠緼udioSpecificConfig?
            byte config1 = (profile<<3)|((sample_rate&0xe)>>1);
            byte config2 = ((sample_rate&0x1)<<7)|(channel<<3);
    AudioSpecificConfig = config1,config2
    */

    unsigned char config1 =
        (AacProfile2FlvAacProfile((Mpeg2AacProfile)profile) << 3) |
        ((sample_rate & 0xe) >> 1);
    unsigned char config2 = ((sample_rate & 0x1) << 7) | (channel << 3);
    // int config2 = ((pstream->samplerate&0xE)<<7)|(pstream->channel<<3);
    commonAssist.putByte(config1);  // AudioSpecificConfig first byte
    commonAssist.putByte(config2);  // AudioSpecificConfig second byte
    commonAssist.putBit24At(commonAssist.getContentLength() - 11,
                            dataSizePos);  // rewrite Data size
    commonAssist.putBit32(commonAssist.getContentLength());  // Pre tag size

    configTagSize = commonAssist.getContentLength();
    mAudioConfigTag = true;
  }

  unsigned long long u64TimeOffset = 0;

  u64TimeOffset = timeStamp;

  commonAssist.putByte(e_FlvTagType_Audio);  // Tag Type
  dataSizePos = commonAssist.getContentLength();
  commonAssist.putBit24(0);  // DataSize(24)

  // 重要，时间戳是AccessProxy音频转码后的 head.state_param().uint64_timestamp()
  commonAssist.putBit24((unsigned int)u64TimeOffset);  // Timestamp(24)
  commonAssist.putByte(((unsigned int)u64TimeOffset) >>
                       24);  // TimestampExtended(8)
  commonAssist.putBit24(0);  // StreamID(24) .

  /*Audio Tag Head*/
  /*
  SoundFormat UB[4] Format of SoundData
          10 = AAC
  SoundRate UB[2] Sampling rate
          0 = 5.5-kHz
          1 = 11-kHz
          2 = 22-kHz
          3 = 44-kHz
          For AAC: always 3
  SoundSize UB[1] Size of each sample
          0 = snd8Bit
          1 = snd16Bit
  SoundType UB[1] Mono or stereo sound
          0 = sndMono
          1 = sndStereo
  For AAC: always 1
  */
  commonAssist.putByte(0xAF);  // For AAC: always 0xAF
  commonAssist.putByte(
      1);  // AAC Packet Type: 0 = AAC sequence header; 1 = AAC raw

  commonAssist.putBytes(
      static_cast<const uint8_t*>(mediaBuffer.getSrcBuffer()) +
          AAC_ADTS_HEADER_LENGTH,
      mediaBuffer.getSrcContentLength() - AAC_ADTS_HEADER_LENGTH);  // Data

  commonAssist.putBit24At(commonAssist.getContentLength() - 11 - configTagSize,
                          dataSizePos);  // rewrite Data size
  commonAssist.putBit32(commonAssist.getContentLength() -
                        configTagSize);  // Pre tag size

  mediaBuffer.setDestContentLength(commonAssist.getContentLength());

  return 0;
}

FlvVideoFrameType FlvAssist::AvcFrameType2FlvVideoFrameType(
    const enVideoH264NaluType avcFrameType) {
  switch (avcFrameType) {
    case kVideoNaluIdr:
    case kVideoNaluSps:
    case kVideoNaluPps:
      return e_FlvVideoFrameType_Key;

    case kVideoNaluSlice:
    default:
      return e_FlvVideoFrameType_Inter;
  }
}

FlvAacProfile FlvAssist::AacProfile2FlvAacProfile(
    const Mpeg2AacProfile aacProfile) {
  switch (aacProfile) {
    case e_AacProfile_Main:
      return e_FlvAacProfile_Main;

    case e_AacProfile_LC:
      return e_FlvAacProfile_LC;

    case e_AacProfile_SSR:
      return e_FlvAacProfile_SBR;

    default:
      return e_FlvAacProfile_None;
  }
}

int FlvAssist::toAvc(MediaBuffer& mediaBuffer) {
  int ret = mediaBuffer.checkBuf();
  if (ret != 0) {
    return ret;
  }

  if (mediaBuffer.getSrcContentLength() <= 4) {
    return -1;
  }

  if (memcmp(mediaBuffer.getSrcBuffer(), sFrameStartCode,
             sizeof(sFrameStartCode)) != 0 &&
      memcmp(mediaBuffer.getSrcBuffer(), sSliceStartCode,
             sizeof(sSliceStartCode)) != 0) {
    return 0;
  }

  CommonAssist commonAssist;
  commonAssist.setupBuffer(mediaBuffer.getDestBuffer(),
                           mediaBuffer.getDestBufferLength());

  // Skip over any input bytes that precede the first 0x00000001 or 0x000001:
  unsigned int curParsePos = 0;
  unsigned int contentLen = mediaBuffer.getSrcContentLength();
  const uint8_t* buffer =
      static_cast<const uint8_t*>(mediaBuffer.getSrcBuffer());
  unsigned int first4Bytes = 0;
  while (curParsePos <= contentLen - 4) {
    first4Bytes = (buffer[curParsePos] << 24) |
                  (buffer[curParsePos + 1] << 16) |
                  (buffer[curParsePos + 2] << 8) | (buffer[curParsePos + 3]);
    if ((first4Bytes == 0x00000001 &&
         (buffer[curParsePos + 4] & 0x1F) != 0x09) ||
        ((first4Bytes & 0xFFFFFF00) == 0x00000100 &&
         (buffer[curParsePos + 3] & 0x1F) != 0x09)) {
      break;
    }
    ++curParsePos;
  }

  if (first4Bytes == 0x00000001) {
    curParsePos += 4;  // skip this initial code
  } else if ((first4Bytes & 0xFFFFFF00) == 0x00000100) {
    curParsePos += 3;  // skip this initial code
  } else {
    return -1;
  }

  unsigned char firstByteOfNALUnit = 0;
  unsigned int nalUnitLen = 0;
  unsigned int nalUnitLenPos = 0;
  unsigned char nalUnitType = 0;
  unsigned char firstNalUnitType = 0;
  bool haveSeenFirstNalUnitType = false;
  bool haveSeenEOF = false;
  unsigned int next4Bytes = 0;

  if (contentLen - curParsePos < 4) {
    firstByteOfNALUnit = buffer[curParsePos];
    nalUnitType = firstByteOfNALUnit & 0x1F;
    nalUnitLen = contentLen - curParsePos;
    if (!isSei(nalUnitType) && !isSps(nalUnitType) && !isPps(nalUnitType)) {
      commonAssist.putBit32(nalUnitLen);
    }

    commonAssist.putBytes(buffer + curParsePos, nalUnitLen);
    curParsePos += nalUnitLen;
  } else {
    do {
      if (contentLen < 4 + curParsePos) {
        break;
      }

      nalUnitLen = 0;
      next4Bytes = (buffer[curParsePos] << 24) |
                   (buffer[curParsePos + 1] << 16) |
                   (buffer[curParsePos + 2] << 8) | (buffer[curParsePos + 3]);
      if (next4Bytes == 0x00000001 || (next4Bytes & 0xFFFFFF00) == 0x00000100) {
        return -1;
      }

      firstByteOfNALUnit = next4Bytes >> 24;
      nalUnitType = firstByteOfNALUnit & 0x1F;
      if (!haveSeenFirstNalUnitType) {
        firstNalUnitType = nalUnitType;
        haveSeenFirstNalUnitType = true;
      }

      if (!isSei(nalUnitType) && !isSps(nalUnitType) && !isPps(nalUnitType)) {
        nalUnitLenPos = commonAssist.getContentLength();
        commonAssist.putBit32(0);
      }

      while (next4Bytes != 0x00000001 &&
             (next4Bytes & 0xFFFFFF00) != 0x00000100) {
        if ((unsigned)(next4Bytes & 0xFF) > 1) {
          // Common case: 0x00000001 or 0x000001 definitely doesn't begin
          // anywhere in "next4Bytes", so we save all of it:
          commonAssist.putBytes(buffer + curParsePos, 4);
          curParsePos += 4;
          nalUnitLen += 4;
        } else {
          // Save the first byte, and continue testing the rest:
          next4Bytes >>= 24;
          commonAssist.putByte(buffer[curParsePos]);
          ++curParsePos;
          ++nalUnitLen;
        }

        if (contentLen < 4 + curParsePos) {
          int len = contentLen - curParsePos;
          commonAssist.putBytes(buffer + curParsePos, len);
          nalUnitLen += len;
          haveSeenEOF = true;
          break;
        } else {
          next4Bytes =
              (buffer[curParsePos] << 24) | (buffer[curParsePos + 1] << 16) |
              (buffer[curParsePos + 2] << 8) | (buffer[curParsePos + 3]);
        }
      }

      if (!isSei(nalUnitType) && !isSps(nalUnitType) && !isPps(nalUnitType)) {
        commonAssist.putBit32At(nalUnitLen, nalUnitLenPos);
      }

      if (haveSeenEOF) {
        break;
      }

      // Assert: next4Bytes starts with 0x00000001 or 0x000001, and we've saved
      // all previous bytes (forming a complete NAL unit).
      // Skip over these remaining bytes, up until the start of the next NAL
      // unit:
      if (next4Bytes == 0x00000001) {
        curParsePos += 4;
      } else {
        curParsePos += 3;
      }

      if (isSei(nalUnitType) || isSps(nalUnitType) || isPps(nalUnitType) ||
          nalUnitType == 10 ||
          nalUnitType == 11)  // "end of sequence" or "end of (bit)stream"
      {
        break;
      } else if (nalUnitType != 9 &&
                 !(nalUnitType >= 14 && nalUnitType <= 18)) {
        if (contentLen - curParsePos < 2) {
          break;
        }

        // We need to check the *next* NAL unit to figure out whether
        // the current NAL unit ends an 'access unit':
        unsigned char firstBytesOfNextNALUnit[2];
        firstBytesOfNextNALUnit[0] = buffer[curParsePos];
        firstBytesOfNextNALUnit[1] = buffer[curParsePos + 1];

        unsigned char const& nextNalUnitType =
            (firstBytesOfNextNALUnit[0] & 0x1F);
        if ((nextNalUnitType <= 5 && nextNalUnitType > 0)) {
          // The high-order bit of the byte after the "nal_unit_header" tells us
          // whether it's
          // the start of a new 'access unit' (and thus the current NAL unit
          // ends an 'access unit'):
          unsigned char const byteAfterNalUnitHeader =
              firstBytesOfNextNALUnit[1];
          if ((byteAfterNalUnitHeader & 0x80) != 0) {
            break;
          }
        } else if ((nextNalUnitType >= 6 && nextNalUnitType <= 9) ||
                   (nextNalUnitType >= 14 &&
                    nextNalUnitType <=
                        18))  // These NAL units usually *begin* an access unit
        {
          // The next NAL unit's type is one that usually appears at the start
          // of an 'access unit',
          // so we assume that the current NAL unit ends an 'access unit':
          break;
        }
      }
    } while (1);
  }

  mediaBuffer.setDestContentLength(commonAssist.getContentLength());

  mContext.mFrameType = (enVideoH264NaluType)firstNalUnitType;

  if (isSps(firstNalUnitType)) {
    mContext.mSpsLength =
        (mediaBuffer.getDestContentLength() <= sizeof(mContext.mSps)
             ? mediaBuffer.getDestContentLength()
             : sizeof(mContext.mSps));
    memmove(mContext.mSps, mediaBuffer.getDestBuffer(), mContext.mSpsLength);

    if (!haveSeenEOF) {
      int startCodeLen = 0;
      if (next4Bytes == 0x00000001) {
        startCodeLen = 4;
      } else {
        startCodeLen = 3;
      }

      MediaBuffer tmpMediaBuffer;
      int offset = curParsePos - startCodeLen;
      tmpMediaBuffer.setSrcBuffer(
          static_cast<const uint8_t*>(mediaBuffer.getSrcBuffer()) + offset);
      tmpMediaBuffer.setSrcBufferLength(mediaBuffer.getSrcBufferLength());
      tmpMediaBuffer.setSrcContentLength(mediaBuffer.getSrcContentLength() -
                                         offset);
      tmpMediaBuffer.setDestBuffer(mediaBuffer.getDestBuffer());
      tmpMediaBuffer.setDestBufferLength(mediaBuffer.getDestBufferLength());
      toAvc(tmpMediaBuffer);
      mediaBuffer.setDestContentLength(tmpMediaBuffer.getDestContentLength());
    }
  } else if (isPps(firstNalUnitType)) {
    mContext.mPpsLength =
        (mediaBuffer.getDestContentLength() <= sizeof(mContext.mPps)
             ? mediaBuffer.getDestContentLength()
             : sizeof(mContext.mPps));
    memmove(mContext.mPps, mediaBuffer.getDestBuffer(), mContext.mPpsLength);

    if (!haveSeenEOF) {
      int startCodeLen = 0;
      if (next4Bytes == 0x00000001) {
        startCodeLen = 4;
      } else {
        startCodeLen = 3;
      }

      MediaBuffer tmpMediaBuffer;
      int offset = curParsePos - startCodeLen;
      tmpMediaBuffer.setSrcBuffer(
          static_cast<const uint8_t*>(mediaBuffer.getSrcBuffer()) + offset);
      tmpMediaBuffer.setSrcBufferLength(mediaBuffer.getSrcBufferLength());
      tmpMediaBuffer.setSrcContentLength(mediaBuffer.getSrcContentLength() -
                                         offset);
      tmpMediaBuffer.setDestBuffer(mediaBuffer.getDestBuffer());
      tmpMediaBuffer.setDestBufferLength(mediaBuffer.getDestBufferLength());
      toAvc(tmpMediaBuffer);
      mediaBuffer.setDestContentLength(tmpMediaBuffer.getDestContentLength());
    }
  } else if (isSei(firstNalUnitType)) {
    if (!haveSeenEOF) {
      int startCodeLen = 0;
      if (next4Bytes == 0x00000001) {
        startCodeLen = 4;
      } else {
        startCodeLen = 3;
      }

      MediaBuffer tmpMediaBuffer;
      int offset = curParsePos - startCodeLen;
      tmpMediaBuffer.setSrcBuffer(
          static_cast<const uint8_t*>(mediaBuffer.getSrcBuffer()) + offset);
      tmpMediaBuffer.setSrcBufferLength(mediaBuffer.getSrcBufferLength());
      tmpMediaBuffer.setSrcContentLength(mediaBuffer.getSrcContentLength() -
                                         offset);
      tmpMediaBuffer.setDestBuffer(mediaBuffer.getDestBuffer());
      tmpMediaBuffer.setDestBufferLength(mediaBuffer.getDestBufferLength());
      toAvc(tmpMediaBuffer);
      mediaBuffer.setDestContentLength(tmpMediaBuffer.getDestContentLength());
    }
  }

  return 0;
}

bool FlvAssist::isSei(unsigned char type) { return type == 6 ? true : false; }

bool FlvAssist::isSps(unsigned char type) { return type == 7 ? true : false; }

bool FlvAssist::isPps(unsigned char type) { return type == 8 ? true : false; }

int FlvAssist::makeAacConfTag(MediaBuffer& mediaBuffer) {
  // FLV Tag
  // | TagType(8) | DataSize(24) | Timestamp(24) | TimestampExtended(8) |
  // StreamID(24) |
  // Audio Tag
  // | Sound Format(4) | SoundRate(2) | SoundSize(1) | SoundType(1) |
  // Aac packet
  // AACPackType(8) |
  // AAC Sequence Header or AAC Raw
  //     AAC Sequence Header
  // | audioObjectType(5) | samplingFrequencyIndex(4) | channelConfiguration(4)
  // | frameLengthFlag(1) | dependsOnCoreCoder(1) | extensionFlag(1) |
  //     AAC Raw
  // | AAC Data
  // | PreviousTagSize(32)

  //首先判断是否带ADTS头
  const uint8_t* srcBuffer =
      static_cast<const uint8_t*>(mediaBuffer.getSrcBuffer());
  if (!(srcBuffer[0] == 0xFF && (srcBuffer[1] & 0xF6) == 0xF0))  // from faad
  {
    return -1;
  }

  int dataSizePos = 0;
  unsigned long long u64TimeOffset = 0;
  unsigned int timeStamp = mediaBuffer.getMediaTime();
  CommonAssist commonAssist;
  commonAssist.setupBuffer(mediaBuffer.getDestBuffer(),
                           mediaBuffer.getDestBufferLength());

  u64TimeOffset = timeStamp;

  commonAssist.putByte(e_FlvTagType_Audio);  // Tag Type
  dataSizePos = commonAssist.getContentLength();
  commonAssist.putBit24(0);  // DataSize(24)
  // commonAssist.putBit24(timeStamp);         // Timestamp(24)
  // commonAssist.putByte(timeStamp >> 24);    // TimestampExtended(8)
  commonAssist.putBit24((unsigned int)u64TimeOffset);  // Timestamp(24)
  commonAssist.putByte(((unsigned int)u64TimeOffset) >>
                       24);  // TimestampExtended(8)
  commonAssist.putBit24(0);  // StreamID(24)

  /*Audio Tag Head*/
  /*
  SoundFormat UB[4] Format of SoundData
          10 = AAC
  SoundRate UB[2] Sampling rate
          0 = 5.5-kHz
          1 = 11-kHz
          2 = 22-kHz
          3 = 44-kHz
          For AAC: always 3
  SoundSize UB[1] Size of each sample
          0 = snd8Bit
          1 = snd16Bit
  SoundType UB[1] Mono or stereo sound
          0 = sndMono
          1 = sndStereo
  For AAC: always 1
  */
  commonAssist.putByte(0xAF);  // For AAC: always 0xAF
  commonAssist.putByte(
      0);  // AAC Packet Type: 0 = AAC sequence header; 1 = AAC raw
  // if(AAC Packet Type == 0) AudioSpecificConfig
  /*
  AudioSpecificConfig靠靠縄SO
  14496-3縣ttp://www.nhzjj.com/asp/admin/editor/newsfile/2010318163752818.pdf?
          audioObjectType:         5 bits?緼AC-LC?縎BR?9縋S
          samplingFrequencyIndex:  4 bits靠靠靠靠靠
          channelConfiguration:    4 bits靠靠
          if (audioObjectType == 5 || audioObjectType == 29)
                  extensionSamplingFrequencyIndex: 4 bits靠靠靠靠?
                  audioObjectType:                 5 bits靠靠靠緼OT
          GASpecificConfig
          frameLengthFlag:         1 bit?靠靠1024?靠靠960
          DependsOnCoreCoder:      1 bit
          extensionFlag:           1 bit

  AudioSpecificConfig靠靠ADTS?靠靠靠靠3靠靠?
          byte profile(LC,Main,HE) = ((payload[2]&0xc0)>>6)+1;
          byte sample_rate         = (payload[2]&0x3c)>>2;
          byte channel             =
  ((payload[2]&0x1)<<2)|((payload[3]&0xc0)>>6);
  靠靠3靠靠靠2靠緼udioSpecificConfig?
          byte config1 = (profile<<3)|((sample_rate&0xe)>>1);
          byte config2 = ((sample_rate&0x1)<<7)|(channel<<3);
  AudioSpecificConfig = config1,config2
  */
  unsigned char profile = ((srcBuffer[2] & 0xc0) >> 6);
  unsigned char sample_rate = (srcBuffer[2] & 0x3c) >> 2;
  unsigned char channel =
      ((srcBuffer[2] & 0x1) << 2) | ((srcBuffer[3] & 0xc0) >> 6);

  unsigned char config1 =
      (AacProfile2FlvAacProfile((Mpeg2AacProfile)profile) << 3) |
      ((sample_rate & 0xe) >> 1);
  unsigned char config2 = ((sample_rate & 0x1) << 7) | (channel << 3);
  // int config2 = ((pstream->samplerate&0xE)<<7)|(pstream->channel<<3);
  commonAssist.putByte(config1);  // AudioSpecificConfig first byte
  commonAssist.putByte(config2);  // AudioSpecificConfig second byte
  commonAssist.putBit24At(commonAssist.getContentLength() - 11,
                          dataSizePos);                    // rewrite Data size
  commonAssist.putBit32(commonAssist.getContentLength());  // Pre tag size

  mediaBuffer.setDestContentLength(commonAssist.getContentLength());

  return 0;
}

int FlvAssist::makeAvcConfTag(MediaBuffer& mediaBuffer, int& tagSize) {
  // FLV Tag
  // | TagType(8) | DataSize(24) | Timestamp(24) | TimestampExtended(8) |
  // StreamID(24) |
  // Video Tag
  // | FrameType(4) | CodecID(4) | AVCPackType(8) | CompositionTime(24) |
  // Avc Packet
  // | AVC Nalu Lenght(32) | AVC Nalu ES |
  // | PreviousTagSize(32)

  /*VIDEO_TAG*/
  int ret = 0;

  int dataSizePos = 0;
  int configTagSize = 0;
  unsigned int timeStamp = mediaBuffer.getMediaTime();
  CommonAssist commonAssist;
  unsigned long long u64TimeOffset = 0;

  commonAssist.setupBuffer(mediaBuffer.getDestBuffer(),
                           mediaBuffer.getDestBufferLength());

  // TODO：如果sps和pps不是相邻而来，或者pps先于sps而来，是否需要处理？
  if (mContext.mSpsLength > 0 && mContext.mPpsLength > 0) {
    u64TimeOffset = timeStamp;
    commonAssist.putByte(e_FlvTagType_Video);  // Tag Type

    dataSizePos = commonAssist.getContentLength();
    commonAssist.putBit24(0);                   // DataSize(24)
    commonAssist.putBit24(u64TimeOffset);       // Timestamp(24)
    commonAssist.putByte(u64TimeOffset >> 24);  // TimestampExtended(8)
    commonAssist.putBit24(0);                   // StreamID(24), always 0

    commonAssist.putByte(e_FlvVideoFrameType_Key |
                         e_FlvVideoCodecId_Avc);      // Frametype and CodecID
    commonAssist.putByte(e_FlvAvcPacketType_Header);  // AVCPacketType
    commonAssist.putBit24(0);                         // composition time

    /*
    AVCDecorderConfigurationRecord包括文件的信息。
    具体格式如下：
    | cfgVersion(8) | avcProfile(8) | profileCompatibility(8) |avcLevel(8) |
    | reserved(6) | lengthSizeMinusOne(2) | reserved(3) | numOfSPS(5)
    |spsLength(16) | sps(n) | numOfPPS(8) | ppsLength(16) | pps(n) |
    */

    commonAssist.putByte(1);                 // version
    commonAssist.putByte(mContext.mSps[1]);  // profile
    commonAssist.putByte(mContext.mSps[2]);  // profile
    commonAssist.putByte(mContext.mSps[3]);  // level
    commonAssist.putByte(
        0xff);  // 6 bits reserved (111111) + 2 bits nal size length - 1 (11)
    commonAssist.putByte(
        0xe1);  // 3 bits reserved (111) + 5 bits number of sps (00001)

    commonAssist.putBit16(mContext.mSpsLength);                 // sps length
    commonAssist.putBytes(mContext.mSps, mContext.mSpsLength);  // sps data

    commonAssist.putByte(1);                                    // number of pps
    commonAssist.putBit16(mContext.mPpsLength);                 // pps length
    commonAssist.putBytes(mContext.mPps, mContext.mPpsLength);  // sps data

    commonAssist.putBit24At(commonAssist.getContentLength() - 11,
                            dataSizePos);  // rewrite Data size
    commonAssist.putBit32(commonAssist.getContentLength());  // Pre tag size

    configTagSize = commonAssist.getContentLength();
    tagSize = configTagSize;

    // CallbackLog(_LEVEL_DEBUG_, push_key.uin, "",
    //    "channel=%llu, make video flv config tag, startVideoTime=%u,
    //    pose_time=%u",
    //    channelId,mStartTimeVid, timeStamp-mStartTimeVid);
  } else {
    tagSize = configTagSize;
    return 0;  // 忽略该数据（可能是先来的P帧、重复的sps等）
  }

  mediaBuffer.setDestContentLength(commonAssist.getContentLength());

  // CallbackLog(_LEVEL_DEBUG_, push_key.uin,"",
  //        "Make video conf tag,
  //        Channel=%llu,Len=%u,FlagTime=%u,StartTime=%u,CurTime=%u",
  //        channelId, mediaBuffer.getDestContentLength(), u64TimeOffset,
  //        mStartTimeVid, timeStamp);

  return ret;
}
}  // namespace tywebrtc
