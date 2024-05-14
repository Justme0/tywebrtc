// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTMP_COMMONASSIST_H_
#define SRC_RTMP_COMMONASSIST_H_

#include <cstdint>

namespace tywebrtc {

class CommonAssist {
 public:
  CommonAssist(void);
  ~CommonAssist(void);

 public:
  int setupBuffer(void* buffer, const unsigned int bufferLen);
  unsigned int getContentLength();

  int putByte(const unsigned char val);
  int putBit16(const unsigned short val);
  int putBit24(const unsigned int val);
  int putBit32(const unsigned int val);
  int putBit64(const unsigned long long val);
  int putBytes(const void* data, unsigned int length);

  int putByteAt(const unsigned char val, const unsigned int pos);
  int putBit24At(const unsigned int val, const unsigned int pos);
  int putBit32At(const unsigned int val, const unsigned int pos);

 private:
  uint8_t* mBuffer;
  unsigned int mBufferLength;
  unsigned int mCurrentPos;
};

}  // namespace tywebrtc

#endif  // SRC_RTMP_COMMONASSIST_H_
