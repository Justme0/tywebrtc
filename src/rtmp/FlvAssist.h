// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTMP_FLVASSIST_H_
#define SRC_RTMP_FLVASSIST_H_

#include <arpa/inet.h>

#include <vector>

#include "src/rtmp/MediaBuffer.h"

namespace tywebrtc {

#define AAC_ADTS_HEADER_LENGTH 7
#define DST_BUF_LEN (2048 * 1024)

class RtmpPusher;

class FlvAssist {
 public:
  FlvAssist(RtmpPusher& belongingRtmpHandler);

  int SendVideoFrame(const std::vector<char>& h264Frame, uint64_t frameMs);
  int SendAudioFrame(const std::vector<char>& audioFrame, uint64_t frameMs);

  FlvVideoFrameType AvcFrameType2FlvVideoFrameType(
      const EnVideoH264NaluType avcFrameType);
  FlvAacProfile AacProfile2FlvAacProfile(const Mpeg2AacProfile aacProfile);

  int toAvc(MediaBuffer& mediaBuffer);
  int CheckNalu(unsigned char* buffer, unsigned int buffer_len);
  bool isSei(unsigned char type);
  bool isSps(unsigned char type);
  bool isPps(unsigned char type);

  int makeAvcConfTag(MediaBuffer& mediaBuffer, int& tagSize);
  int makeAacConfTag(MediaBuffer& mediaBuffer);

  void Init();

 private:
  int makeTag(MediaBuffer& mediaBuffer, MediaType mediaType);
  int makeAvcTag(MediaBuffer& mediaBuffer);
  int makeAacTag(MediaBuffer& mediaBuffer);

 public:
  RtmpPusher& belongingRtmpHandler_;

  static const unsigned char sFrameStartCode[];
  static const unsigned char sSliceStartCode[];
  uint32_t mStartTimeAud;
  uint32_t mStartTimeVid;
  bool mVideoConfigTag;  // 是否已有config tag
  bool mAudioConfigTag;  // 是否已有config tag

  unsigned char m_LastSampleRate;
  unsigned int mAudioFramecnt;
  H264Context mContext;
};

}  // namespace tywebrtc

#endif  // SRC_RTMP_FLVASSIST_H_