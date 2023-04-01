#ifndef __COMMON_ASSIST_H__
#define __COMMON_ASSIST_H__

#include <cstdint>

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

#endif
