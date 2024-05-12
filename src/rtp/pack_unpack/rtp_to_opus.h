// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_PACK_UNPACK_RTP_TO_OPUS_H_
#define SRC_RTP_PACK_UNPACK_RTP_TO_OPUS_H_

#include <cstdint>

namespace tywebrtc {

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

}  // namespace tywebrtc

#endif  // SRC_RTP_PACK_UNPACK_RTP_TO_OPUS_H_
