#ifndef RTP_PACK_UNPACK_RTP_TO_OPUS_H_
#define RTP_PACK_UNPACK_RTP_TO_OPUS_H_

#include <cstdint>

class RtpDepacketizerOpus {
 public:
  RtpDepacketizerOpus();

  explicit RtpDepacketizerOpus(uint8_t audio_level_index);

  void ReInit(uint8_t audio_level_index) {
    audio_level_index_ = audio_level_index;
  };

  int Process(uint8_t* data, uint32_t length);
  int ProcessExtInfo(uint8_t* data, int length);
  int ParseAudioLevel(uint8_t* data, uint32_t length, uint32_t& audio_level);

 private:
  void DecodeOneByteExt(const uint8_t* pStart, const uint8_t* pExtEnd);

 private:
  uint8_t audio_level_index_;
  uint32_t audio_level_;
};

#endif  // RTP_PACK_UNPACK_RTP_TO_OPUS_H_
