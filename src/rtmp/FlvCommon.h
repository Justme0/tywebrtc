#ifndef __FLV_COMMON_H__
#define __FLV_COMMON_H__

#include <cstdlib>
#include <string>

enum MediaType {
  e_MediaType_None = 0,
  e_MediaType_raw,
  e_MediaType_Avc,
  e_MediaType_Aac,
  e_MediaType_Pcm,
  e_MediaType_Flv
};

enum Mpeg2AacProfile {
  e_AacProfile_Main = 0,
  e_AacProfile_LC = 1,
  e_AacProfile_SSR = 2,
  e_AacProfile_Reserved = 3
};

enum FlvTagType {
  e_FlvTagType_None = 0,
  e_FlvTagType_Audio = 0x08,  // audio
  e_FlvTagType_Video = 0x09,  // video
  e_FlvTagType_Meta = 0x12    // script data
};

enum FlvAudioSoundFormat {
  e_FlvAudioSoundFormat_None = -1,
  e_FlvAudioSoundFormat_Pcm = 0,
  e_FlvAudioSoundFormat_Adpcm = 1,
  e_FlvAudioSoundFormat_Mp3 = 2,
  e_FlvAudioSoundFormat_PcmLittleEnd = 3,
  e_FlvAudioSoundFormat_Nelly16K = 4,
  e_FlvAudioSoundFormat_Nelly8K = 5,
  e_FlvAudioSoundFormat_Nelly = 6,
  e_FlvAudioSoundFormat_G711a = 7,
  e_FlvAudioSoundFormat_G711u = 8,
  e_FlvAudioSoundFormat_Reserved = 9,
  e_FlvAudioSoundFormat_Aac = 10,
  e_FlvAudioSoundFormat_Speex = 11,
  e_FlvAudioSoundFormat_Mp3_8k = 14,
  e_FlvAudioSoundFormat_DeviceSpecific = 15
};

enum FlvAudioSoundRate {
  e_FlvAudioSoundRate_None = -1,
  e_FlvAudioSoundRate_5Dot5K = 0,
  e_FlvAudioSoundRate_11K = 1,
  e_FlvAudioSoundRate_22K = 2,
  e_FlvAudioSoundRate_44K = 3
};

enum FlvAudioSoundSize {
  e_FlvAudioSoundSize_None = -1,
  e_FlvAudioSoundSize_8Bit = 0,
  e_FlvAudioSoundSize_16Bit = 1
};

enum FlvAudioSoundType {
  e_FlvAudioSoundType_None = -1,
  e_FlvAudioSoundType_Mono = 0,   // For Nellymoser: always 0
  e_FlvAudioSoundType_Stereo = 1  // For AAC: always 1
};

enum FlvAacPacketType {
  e_FlvAacPacketType_None = -1,
  e_FlvAacPacketType_Header = 0,  // AAC sequence header
  e_FlvAacPacketType_Raw = 1      // AAC raw
};

/*
A media type of mp4a (0x6D703461) indicates that the track is encoded with AAC
audio.
Flash Player supports the following AAC profiles, denoted by their object types:
- 1 = main profile
- 2 = low complexity, a.k.a. LC
- 5 = high efficiency/scale band replication, a.k.a. HE/SBR
When the audio codec is AAC, an esds box occurs inside the stsd box of a sample
table.
This box contains initialization data that an AAC decoder requires to decode the
stream.
See ISO/IEC 14496-3 for more information about the structure of this box.
*/
enum FlvAacProfile {
  e_FlvAacProfile_None = 0,
  e_FlvAacProfile_Main = 1,
  e_FlvAacProfile_LC = 2,
  e_FlvAacProfile_SBR = 5
};

enum FlvVideoFrameType {
  e_FlvVideoFrameType_None = 0,
  e_FlvVideoFrameType_Key = 1 << 4,  // key frame(for avc, a seekable frame)
  e_FlvVideoFrameType_Inter =
      2 << 4,  // inter frame(for avc, a non-seekable frame)
  e_FlvVideoFrameType_Disposable = 3
                                   << 4,  // disposable inter frame(H.263 only)
  e_FlvVideoFrameType_Generated =
      4 << 4,  // generated keyframe(reserved for server use only)
  e_FlvVideoFrameType_InfoCmd = 5 << 4  // video info/command frame
};

enum FlvVideoCodecId {
  e_FlvVideoCodecId_None = 0,
  e_FlvVideoCodecId_Jpeg = 1,
  e_FlvVideoCodecId_H263 = 2,
  e_FlvVideoCodecId_Screen = 3,
  e_FlvVideoCodecId_Vp6 = 4,
  e_FlvVideoCodecId_Vp6Alpha = 5,
  e_FlvVideoCodecId_ScreenV2 = 6,
  e_FlvVideoCodecId_Avc = 7,
};

enum FlvAvcPacketType {
  e_FlvAvcPacketType_None = -1,
  e_FlvAvcPacketType_Header = 0,
  e_FlvAvcPacketType_Nalu = 1,
  e_FlvAvcPacketType_End = 2
};

#pragma pack(1)
typedef struct Flv_header {
  unsigned char signature[3]; /* always "FLV" */
  unsigned char version;      /* should be 1 */
  unsigned char flags;
  unsigned int offset; /* always 9 */
} flv_header;
#pragma pack()

#endif
