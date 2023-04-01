#ifndef RTP_PACK_UNPACK_RTP_TO_VP8_H_
#define RTP_PACK_UNPACK_RTP_TO_VP8_H_

#include "codec/video_codec.h"
#include "rtp/rtp_parser.h"

// tmp
typedef struct tagVideoVp8UnPackParam {
  unsigned int TimeStmp;
  unsigned int PackSeqNo;
  unsigned int Ssrc;
  uint16_t PayLoad;
  bool RequestKeyFrame;
} VIDEO_VP8_UNPACK_PARAMS;

class RtpDepacketizerVp8 {
 public:
  RtpDepacketizerVp8();
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
  unsigned long long m_ChanlNo; /* 通道号，由应用层设置，必须设置 */
  unsigned int m_MaxPackSize; /* 最大包长，字节单位，包括RTP头的字节长度 */

  RTP_HEADER_INFO_VP8 m_RtpHeadInfoVp8;

  /* 将数据包的RTP header信息保存，用于校验包打包使用 */
  RtpHeader m_RtpHeader;  // vp8 payload type must set

  int Width_ = 0;
  int Height_ = 0;

  char m_RtpHeadVp8[VP8_MAX_PAYLOAD_HEAD_LEN];
  int m_RtpHeadVp8Len;
  CodecDecoder* decoder;
  CodecEncoder* encoder;
  // VIDEO_FAST_UP_DATE m_pFastUpDate;
  VIDEO_VP8_UNPACK_PARAMS m_UnPackParams;
  char m_Vp8RawData[VP8_RAW_DATA_LEN];
  int m_RawDataLen;
  bool is_key_frame;
  uint32_t encoder_width;
  uint32_t encoder_height;
  uint64_t last_receive_frame_time;
  void* m_pfOutfpH264;
};

#endif  // RTP_PACK_UNPACK_RTP_TO_VP8_H_
