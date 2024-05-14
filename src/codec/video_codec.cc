// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/codec/video_codec.h"

#include <sstream>

#include "src/log/log.h"

namespace tywebrtc {

// Tinyid for FFmpeg log to ULS
// VideoRfc7741::VideoUnPackVp8RtpStm() is single thread, so use global tinyid
// is no problem.
// FFmpeg cannot deliver private data to log callback function, we use global
// variable.
// uint64_t g_ffmpeg_codec_tinyid_for_log = 0;

// for Decoder
CodecDecoder::CodecDecoder() {
  av_codec_tx_ = NULL;
  av_frame_ = NULL;
  init_sucess_ = false;
  sws_context_ = NULL;
  dst_frame_ = NULL;
  width_ = 0;
  height_ = 0;
}
CodecDecoder::~CodecDecoder() {
  if (av_frame_) {
    av_frame_free(&av_frame_);
    av_frame_ = NULL;
  }

  if (av_codec_tx_) {
    avcodec_free_context(&av_codec_tx_);
    av_codec_tx_ = NULL;
  }

  if (dst_frame_) {
    av_freep(&dst_frame_->data[0]);
    av_frame_free(&dst_frame_);
    dst_frame_ = NULL;
  }

  if (sws_context_) {
    sws_freeContext(sws_context_);
    sws_context_ = NULL;
  }
}

bool CodecDecoder::InitDecoder(const CodecParam &param) {
  if (!param.codecName || strcmp(param.codecName, "") == 0) {
    tylog("CodecDecoder codecName is NULL");
    return false;
  }

  const AVCodec *codec = avcodec_find_decoder_by_name(param.codecName);
  if (!codec) {
    tylog("CodecDecoder can't find the decoder,codecName %s", param.codecName);
    return false;
  }

  av_codec_tx_ = avcodec_alloc_context3(NULL);
  if (!av_codec_tx_) {
    tylog("CodecDecoder avcodec_alloc_context3 failed!");
    return false;
  }

  av_codec_tx_->codec_type = codec->type;
  av_codec_tx_->codec_id = codec->id;
  // av_codec_tx_->channels = param.channels;
  // av_codec_tx_->sample_rate = param.sample_rate;
  av_codec_tx_->sample_fmt = AV_SAMPLE_FMT_FLTP;
  av_codec_tx_->pix_fmt = AV_PIX_FMT_YUV420P;
  av_codec_tx_->extradata = NULL;
  av_codec_tx_->extradata_size = 0;
  av_codec_tx_->width = param.width;
  av_codec_tx_->height = param.height;
  // If this is ever increased, look at |av_codec_tx_->thread_safe_callbacks|
  // and
  // make it possible to disable the thread checker in the frame buffer pool.
  av_codec_tx_->thread_count = 1;
  av_codec_tx_->thread_type = FF_THREAD_SLICE;

  int res = avcodec_open2(av_codec_tx_, codec, NULL);
  if (res < 0) {
    tylog("CodecDecoder avcodec_open2 error: %d", res);
    return false;
  }

  av_frame_ = av_frame_alloc();
  init_sucess_ = true;
  return true;
}

AVFrame *CodecDecoder::Decode(uint8_t *encodeData, int len) {
  if (!init_sucess_) {
    tylog("CodecDecoder init_sucess_ failed");
    return NULL;
  }
  AVPacket packet;
  av_init_packet(&packet);
  packet.data = const_cast<uint8_t *>(encodeData);
  packet.size = static_cast<int>(len);

  int result = avcodec_send_packet(av_codec_tx_, &packet);
  if (result < 0) {
    tylog("CodecDecoder avcodec_send_packet error: %d", result);
    return NULL;
  }

  result = avcodec_receive_frame(av_codec_tx_, av_frame_);
  if (result < 0) {
    tylog("CodecDecoder avcodec_receive_frame error: %d", result);
    return NULL;
  }

  return av_frame_;
}

AVFrame *CodecDecoder::ScaleImg(AVFrame *srcFrame, int nDstW, int nDstH) {
  if (dst_frame_ == NULL || nDstW != width_ || nDstH != height_) {
    if (dst_frame_) {
      av_freep(&dst_frame_->data[0]);
      av_frame_free(&dst_frame_);
      dst_frame_ = NULL;
    }
    dst_frame_ = av_frame_alloc();
    if (dst_frame_ == NULL) {
      tylog("ffmpeg av_frame_alloc error!");
      av_frame_unref(srcFrame);
      return NULL;
    }
    dst_frame_->linesize[0] = nDstW;
    dst_frame_->linesize[1] = nDstW >> 1;
    dst_frame_->linesize[2] = nDstW >> 1;
    dst_frame_->width = nDstW;
    dst_frame_->height = nDstH;
    dst_frame_->format = AV_PIX_FMT_YUV420P;
    if (av_image_alloc(dst_frame_->data, dst_frame_->linesize, nDstW, nDstH,
                       AV_PIX_FMT_YUV420P, 16) < 0) {
      tylog("ffmpeg av_image_alloc error!");
      av_frame_unref(srcFrame);
      return NULL;
    }

    if (sws_context_) {
      sws_freeContext(sws_context_);
      sws_context_ = NULL;
    }
    width_ = nDstW;
    height_ = nDstH;
    tylog("ffmpeg ScaleImg src:%dx%d -> dst:%dx%d", srcFrame->width,
          srcFrame->height, nDstW, nDstH);
  }

  sws_context_ = sws_getCachedContext(
      sws_context_, srcFrame->width, srcFrame->height, AV_PIX_FMT_YUV420P,
      dst_frame_->width, dst_frame_->height, AV_PIX_FMT_YUV420P, SWS_BICUBIC,
      NULL, NULL, NULL);

  if (NULL == sws_context_) {
    tylog("ffmpeg get context error!");
    av_frame_unref(srcFrame);
    return NULL;
  }
  int ret = sws_scale(sws_context_, srcFrame->data, srcFrame->linesize, 0,
                      srcFrame->height, dst_frame_->data, dst_frame_->linesize);
  if (ret < 0) {
    tylog("ffmpeg sws_scale error!, ret=%d", ret);
  }
  av_frame_unref(srcFrame);
  return dst_frame_;
}

// for Encoder
CodecEncoder::CodecEncoder() {
  av_codec_tx_ = NULL;
  av_packet_ = NULL;
  init_sucess_ = false;
}
CodecEncoder::~CodecEncoder() { Reset(); }

bool CodecEncoder::InitEncoder(const CodecParam &param) {
  if (!param.codecName || strcmp(param.codecName, "") == 0) {
    tylog("CodecEncoder codecName is NULL");
    return false;
  }

  Reset();

  const AVCodec *codec = avcodec_find_encoder_by_name(param.codecName);
  if (!codec) {
    tylog("CodecEncoder can't find the encoder,codecName %s", param.codecName);
    return false;
  }

  av_codec_tx_ = avcodec_alloc_context3(NULL);
  if (!av_codec_tx_) {
    tylog("CodecEncoder avcodec_alloc_context3 failed!");
    return false;
  }

  av_codec_tx_->codec_type = codec->type;
  av_codec_tx_->codec_id = codec->id;
  av_codec_tx_->pix_fmt = AV_PIX_FMT_YUV420P;
  av_codec_tx_->extradata = NULL;
  av_codec_tx_->extradata_size = 0;

  // If this is ever increased, look at |av_codec_tx_->thread_safe_callbacks|
  // and
  // make it possible to disable the thread checker in the frame buffer pool.
  av_codec_tx_->thread_count = param.thread;
  av_codec_tx_->thread_type = FF_THREAD_SLICE;
  av_codec_tx_->bit_rate = param.bitRate;
  av_codec_tx_->gop_size = 30;

  av_codec_tx_->time_base.num = 1;
  av_codec_tx_->time_base.den = 15;  // taylor to check

  av_codec_tx_->qmin = 10;
  av_codec_tx_->qmax = 51;
  av_codec_tx_->me_range = 16;
  av_codec_tx_->max_qdiff = 4;
  av_codec_tx_->qcompress = 0.6;

  av_codec_tx_->flags |=
      AV_CODEC_FLAG2_LOCAL_HEADER;  // sps pps before every key frame
  // Optional Param
  av_codec_tx_->max_b_frames = 0;
  av_codec_tx_->width = param.width;
  av_codec_tx_->height = param.height;

  if (strcmp(param.codecName, "libx264") == 0) {
    AVDictionary *av_dict = NULL;
    av_dict_set(&av_dict, "preset", param.preset, 0);
    av_dict_set(&av_dict, "tune", param.tune, 0);
    av_dict_set(&av_dict, "profile", param.profile, 0);
    int res = avcodec_open2(av_codec_tx_, codec, &av_dict);
    if (res < 0) {
      tylog("CodecEncoder avcodec_open2 error: %d", res);
      return false;
    }
    av_dict_free(&av_dict);
  } else {
    av_codec_tx_->profile =
        3;  // Bitstream profile number to use; reduce complexity if > 0
    int res = avcodec_open2(av_codec_tx_, codec, NULL);
    if (res < 0) {
      tylog("CodecEncoder avcodec_open2 error: %d", res);
      return false;
    }
  }
  av_packet_ = av_packet_alloc();
  init_sucess_ = true;
  return true;
}

AVPacket *CodecEncoder::Encode(AVFrame *yuvFrame, bool isKeyFrame,
                               bool unref_frame) {
  if (!init_sucess_) {
    tylog("CodecEncoder init_sucess_ failed");
    return NULL;
  }

  if (!yuvFrame) {
    return NULL;
  }

  if (isKeyFrame) {
    yuvFrame->pict_type = AV_PICTURE_TYPE_I;
    yuvFrame->key_frame = 1;
  }

  int result = avcodec_send_frame(av_codec_tx_, yuvFrame);
  if (result < 0) {
    tylog("CodecEncoder avcodec_send_frame error: %d", result);
    return NULL;
  }

  result = avcodec_receive_packet(av_codec_tx_, av_packet_);
  if (result < 0) {
    tylog("CodecEncoder avcodec_receive_packet error: %d", result);
    return NULL;
  }

  if (isKeyFrame) {
    yuvFrame->pict_type = AV_PICTURE_TYPE_NONE;
    yuvFrame->key_frame = 0;
  }
  if (unref_frame) {
    av_frame_unref(yuvFrame);
  }

  return av_packet_;
}

void CodecEncoder::UnrefPacket() {
  if (av_packet_) {
    av_packet_unref(av_packet_);
  }
}

void CodecEncoder::Reset() {
  if (av_packet_) {
    av_packet_free(&av_packet_);
    av_packet_ = NULL;
  }

  if (av_codec_tx_) {
    avcodec_free_context(&av_codec_tx_);
    av_codec_tx_ = NULL;
  }
}

}  // namespace tywebrtc
