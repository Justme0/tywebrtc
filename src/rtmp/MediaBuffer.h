// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTMP_MEDIABUFFER_H_
#define SRC_RTMP_MEDIABUFFER_H_

#include <cstdint>

#include "src/rtmp/FlvCommon.h"
#include "src/rtp/pack_unpack/pack_unpack_common.h"

namespace tywebrtc {

class MediaBuffer {
 public:
  explicit MediaBuffer();

  const void* getSrcBuffer() const;
  void setSrcBuffer(const void* buffer);

  uint32_t getSrcBufferLength();
  uint32_t getSrcBufferLength() const;
  void setSrcBufferLength(const uint32_t length);

  uint32_t getSrcContentLength();
  uint32_t getSrcContentLength() const;
  void setSrcContentLength(const uint32_t length);

  void* getDestBuffer();
  const void* getDestBuffer() const;
  void setDestBuffer(void* buffer);

  uint32_t getDestBufferLength();
  uint32_t getDestBufferLength() const;
  void setDestBufferLength(const uint32_t length);

  uint32_t getDestContentLength();
  uint32_t getDestContentLength() const;
  void setDestContentLength(const uint32_t length);

  uint32_t getMediaTime();
  void setMediaTime(uint32_t mediaTime);

  uint32_t getCompositionTimeOffset();
  void setCompositionTimeOffset(uint32_t timeOffset);

  enVideoH264NaluType getFrameType();
  void setFrameType(enVideoH264NaluType frameType);

  int checkBuf();

  void init(const void* inBuf, uint32_t inBufLen, uint8_t* outBuf,
            uint32_t outBufLen);

 private:
  const void* mSrcBuffer;
  uint32_t mSrcBufferLength;
  uint32_t mSrcContentLength;

  void* mDestBuffer;
  uint32_t mDestBufferLength;
  uint32_t mDestContentLength;

  uint32_t mMediaTime;
  uint32_t mCompositionTimeOffset;  // AVC Video Packet特有，值为PTS -
  // DTS，有符号3字节数（从不为负）
  enVideoH264NaluType mframeType;
};

class H264Context {
 public:
  H264Context();

 public:
  enVideoH264NaluType mFrameType;

  uint8_t mSps[1024];  // 注意：后续优化为动态内存
  uint32_t mSpsLength;

  uint8_t mPps[1024];  // 注意：后续优化为动态内存
  uint32_t mPpsLength;

  bool mDecodeSuccess;
};

}  // namespace tywebrtc

#endif  // SRC_RTMP_MEDIABUFFER_H_
