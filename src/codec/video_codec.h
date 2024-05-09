#ifndef RTMP_VIDEO_CODEC_H_
#define RTMP_VIDEO_CODEC_H_

#include <cstdint>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
};

namespace tywebrtc {

struct CodecParam {
  int bitRate;
  int frameRate;
  int width;
  int height;
  const char* preset;
  const char* tune;
  const char* profile;
  const char* level;
  int thread;
  const char* codecName;

  CodecParam() {
    bitRate = 350000;
    frameRate = 15;
    width = 640;
    height = 480;
    preset = "ultrafast";
    tune = "zerolatency";
    profile = "baseline";
    level = "3.0";
    thread = 1;
    codecName = "libx264";
  }
};

class CodecDecoder {
 public:
  CodecDecoder();
  ~CodecDecoder();
  bool InitDecoder(const CodecParam& param);
  AVFrame* Decode(uint8_t* encodeData, int len);
  AVFrame* ScaleImg(AVFrame* srcFrame, int nDstW, int nDstH);

 private:
  AVCodecContext* av_codec_tx_;
  AVFrame* av_frame_;
  bool init_sucess_;
  struct SwsContext* sws_context_;
  AVFrame* dst_frame_;
  int width_, height_;
};

class CodecEncoder {
 public:
  CodecEncoder();
  ~CodecEncoder();
  bool InitEncoder(const CodecParam& param);
  AVPacket* Encode(AVFrame* yuvFrame, bool isKeyFrame, bool unref_frame);
  void UnrefPacket();
  void Reset();

 private:
  AVCodecContext* av_codec_tx_;
  AVPacket* av_packet_;
  bool init_sucess_;
};

}  // namespace tywebrtc

#endif  // RTMP_VIDEO_CODEC_H_
