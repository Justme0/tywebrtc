// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_PACK_UNPACK_RTP_TO_VP8_H_
#define SRC_RTP_PACK_UNPACK_RTP_TO_VP8_H_

extern "C" {
#include "libavformat/avformat.h"
}

#include "src/codec/video_codec.h"
#include "src/rtp/rtp_parser.h"

namespace tywebrtc {

class RtpHandler;

// tmp
typedef struct tagVideoVp8UnPackParam {
  unsigned int PackSeqNo;
  bool RequestKeyFrame;
} VIDEO_VP8_UNPACK_PARAMS;

class RtpDepacketizerVp8 {
 public:
  explicit RtpDepacketizerVp8(RtpHandler& rtpHandler);
  ~RtpDepacketizerVp8();

  int VideoUnPackVp8RtpStm(const char* pData, int Length,
                           std::vector<std::string>* o_h264Frames);
  static bool IsIdrFrame(char* buf, int buf_len);

 private:
  void InitRTPVideoHeaderVP8();
  void Init();
  void VideoRtpStat(int Id);
  void VideoRtpStat(int Id, int Cnt);

 private:
  RTP_HEADER_INFO_VP8 m_RtpHeadInfoVp8;

  /* 将数据包的RTP header信息保存，用于校验包打包使用 */
  RtpHeader m_RtpHeader;  // vp8 payload type must set

  int Width_ = 0;
  int Height_ = 0;

  char m_RtpHeadVp8[VP8_MAX_PAYLOAD_HEAD_LEN];
  CodecDecoder* decoder;
  CodecEncoder* encoder;

  VIDEO_VP8_UNPACK_PARAMS m_UnPackParams;
  // save one frame data
  char m_Vp8RawData[VP8_RAW_DATA_LEN];
  int m_RawDataLen;
  bool is_key_frame;
  uint32_t encoder_width;
  uint32_t encoder_height;
  uint64_t last_receive_frame_time;
  void* m_pfOutfpH264;
  RtpHandler& belongingRtpHandler_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_PACK_UNPACK_RTP_TO_VP8_H_
