#include "CommonAssist.h"

#include <cstdio>
#include <cstring>

CommonAssist::CommonAssist(void)
    : mBuffer(NULL), mBufferLength(0), mCurrentPos(0) {}

CommonAssist::~CommonAssist(void) {}

int CommonAssist::setupBuffer(void* buffer, const unsigned int bufferLen) {
  if (NULL == buffer || 0 == bufferLen) {
    return -1;
  }

  mBuffer = static_cast<uint8_t*>(buffer);
  mBufferLength = bufferLen;
  mCurrentPos = 0;
  return 0;
}

unsigned int CommonAssist::getContentLength() { return mCurrentPos; }

int CommonAssist::putByte(const unsigned char val) {
  if (NULL == mBuffer || 0 == mBufferLength) {
    return -1;
  }

  if (mCurrentPos + 1 >= mBufferLength) {
    return -2;
  }

  mBuffer[mCurrentPos] = val;
  ++mCurrentPos;

  return 0;
}

int CommonAssist::putBit16(const unsigned short val) {
  return (0 == putByte(val >> 8)) && (0 == putByte(val)) ? 0 : -1;
}

int CommonAssist::putBit24(const unsigned int val) {
  return (0 == putBit16(val >> 8)) && (0 == putByte(val)) ? 0 : -1;
}

int CommonAssist::putBit32(const unsigned int val) {
  return (0 == putByte(val >> 24)) && (0 == putByte(val >> 16)) &&
                 (0 == putByte(val >> 8)) && (0 == putByte(val))
             ? 0
             : 1;
}

int CommonAssist::putBit64(const unsigned long long val) {
  return (0 == putBit32(val >> 32)) && (0 == putBit32(val)) ? 0 : 1;
}

int CommonAssist::putBytes(const void* data, unsigned int length) {
  if (mCurrentPos + length >= mBufferLength) {
    return -1;
  }

  memmove(mBuffer + mCurrentPos, data, length);
  mCurrentPos += length;

  return 0;
}

int CommonAssist::putByteAt(const unsigned char val, const unsigned int pos) {
  if (pos >= mBufferLength) {
    return -1;
  }

  mBuffer[pos] = val;

  return 0;
}

int CommonAssist::putBit24At(const unsigned int val, const unsigned int pos) {
  if (pos + 3 >= mBufferLength) {
    return -1;
  }

  mBuffer[pos] = val >> 16;
  mBuffer[pos + 1] = val >> 8;
  mBuffer[pos + 2] = val;

  return 0;
}

int CommonAssist::putBit32At(const unsigned int val, const unsigned int pos) {
  if (pos + 4 >= mBufferLength) {
    return -1;
  }

  mBuffer[pos] = val >> 24;
  mBuffer[pos + 1] = val >> 16;
  mBuffer[pos + 2] = val >> 8;
  mBuffer[pos + 3] = val;

  return 0;
}
