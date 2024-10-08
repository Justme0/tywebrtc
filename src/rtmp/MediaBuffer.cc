// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "MediaBuffer.h"

#include <cstdlib>
#include <cstring>

namespace tywebrtc {

MediaBuffer::MediaBuffer()
    : mSrcBuffer(NULL),
      mSrcBufferLength(0),
      mSrcContentLength(0),
      mDestBuffer(NULL),
      mDestBufferLength(0),
      mDestContentLength(0),
      mMediaTime(0),
      mCompositionTimeOffset(0) {}

const void* MediaBuffer::getSrcBuffer() const { return mSrcBuffer; }

void MediaBuffer::setSrcBuffer(const void* buffer) { mSrcBuffer = buffer; }

uint32_t MediaBuffer::getSrcBufferLength() { return mSrcBufferLength; }

uint32_t MediaBuffer::getSrcBufferLength() const { return mSrcBufferLength; }

void MediaBuffer::setSrcBufferLength(const uint32_t length) {
  mSrcBufferLength = length;
}

uint32_t MediaBuffer::getSrcContentLength() { return mSrcContentLength; }

uint32_t MediaBuffer::getSrcContentLength() const { return mSrcContentLength; }

void MediaBuffer::setSrcContentLength(const uint32_t length) {
  mSrcContentLength = length;
}

void* MediaBuffer::getDestBuffer() { return mDestBuffer; }

const void* MediaBuffer::getDestBuffer() const { return mDestBuffer; }

void MediaBuffer::setDestBuffer(void* buffer) { mDestBuffer = buffer; }

uint32_t MediaBuffer::getDestBufferLength() { return mDestBufferLength; }

uint32_t MediaBuffer::getDestBufferLength() const { return mDestBufferLength; }

void MediaBuffer::setDestBufferLength(const uint32_t length) {
  mDestBufferLength = length;
}

uint32_t MediaBuffer::getDestContentLength() { return mDestContentLength; }

uint32_t MediaBuffer::getDestContentLength() const {
  return mDestContentLength;
}

void MediaBuffer::setDestContentLength(const uint32_t length) {
  mDestContentLength = length;
}

uint32_t MediaBuffer::getMediaTime() { return mMediaTime; }

void MediaBuffer::setMediaTime(uint32_t mediaTime) { mMediaTime = mediaTime; }

uint32_t MediaBuffer::getCompositionTimeOffset() {
  return mCompositionTimeOffset;
}

void MediaBuffer::setCompositionTimeOffset(uint32_t timeOffset) {
  mCompositionTimeOffset = timeOffset;
}

EnVideoH264NaluType MediaBuffer::getFrameType() { return mframeType; }

void MediaBuffer::setFrameType(EnVideoH264NaluType frameType) {
  mframeType = frameType;
}

int MediaBuffer::checkBuf() {
  if (NULL == mSrcBuffer || 0 == mSrcBufferLength || 0 == mSrcContentLength ||
      NULL == mDestBuffer || 0 == mDestBufferLength ||
      mSrcContentLength > mSrcBufferLength ||
      mDestContentLength > mDestBufferLength) {
    return -1;
  }

  return 0;
}

void MediaBuffer::init(const void* inBuf, uint32_t inBufLen, uint8_t* outBuf,
                       uint32_t outBufLen) {
  mSrcBuffer = inBuf;
  mSrcBufferLength = inBufLen;
  mSrcContentLength = inBufLen;
  mDestBuffer = outBuf;
  mDestBufferLength = outBufLen;
  mDestContentLength = 0;
}

H264Context::H264Context()
    : mFrameType(kVideoNaluUnspecific),
      mSpsLength(0),
      mPpsLength(0),
      mDecodeSuccess(false) {
  memset(mSps, 0, sizeof(mSps));
  memset(mPps, 0, sizeof(mPps));
}

}  // namespace tywebrtc
