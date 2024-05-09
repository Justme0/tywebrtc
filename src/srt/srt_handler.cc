#include "src/srt/srt_handler.h"

#include <cassert>

#include "tylib/time/time_util.h"

#include "src/global_tmp/global_tmp.h"
#include "src/log/log.h"

namespace tywebrtc {

SrtHandler::~SrtHandler() {
  if (nullptr == formatContext_) {
    return;
  }

  av_write_trailer(formatContext_);

  avformat_close_input(&formatContext_);  // ?

  /* close output */
  if (formatContext_ && !(formatContext_->oformat->flags & AVFMT_NOFILE)) {
    avio_closep(&formatContext_->pb);
  }

  avformat_free_context(formatContext_);

  formatContext_ = nullptr;
}

int SrtHandler::InitProtocolHandler(const std::string& srtUrl) {
  int ret = 0;

  avformat_alloc_output_context2(&formatContext_, nullptr, "mpegts",
                                 srtUrl.c_str());
  if (nullptr == formatContext_) {
    tylog("Cannot allocate output context, url=%s.", srtUrl.c_str());

    return -1;
  }

  assert(!(formatContext_->oformat->flags & AVFMT_NOFILE));

  ret = avio_open(&formatContext_->pb, formatContext_->url, AVIO_FLAG_WRITE);
  if (ret < 0) {
    tylog("Cannot open avio ret=%d[%s]", ret, av_err2string(ret));

    // use goto error?
    avformat_free_context(formatContext_);
    formatContext_ = nullptr;

    return ret;
  }

  tylog("connect ok, url=%s.", formatContext_->url);

  AVDictionary* options = nullptr;
  ret = AddAudioStream_(48000, 2, options);
  if (ret) {
    tylog("addAudioStream ret=%d.", ret);
    return ret;
  }

  ret = AddVideoStream_(450, 450, options);
  if (ret) {
    tylog("addVideoStream ret=%d.", ret);
    return ret;
  }

  ret = avformat_write_header(formatContext_, &options);
  if (ret < 0) {
    tylog("write_header ret=%d[%s]", ret, av_err2string(ret));

    avformat_free_context(formatContext_);
    formatContext_ = nullptr;

    return ret;
  }

  return 0;
}

bool SrtHandler::InitSucc() const { return nullptr != this->formatContext_; }

int SrtHandler::AddAudioStream_(uint32_t sampleRate, uint32_t channels,
                                AVDictionary*& options) {
  int ret = 0;

  const enum AVCodecID codec_id = AV_CODEC_ID_AAC;
  AVStream* stream = avformat_new_stream(formatContext_, NULL);
  if (nullptr == stream) {
    tylog("Cannot add audio stream");

    return -1;
  }

  stream->time_base = AVRational{1, 90000};
  AVCodecParameters* par = stream->codecpar;
  par->format = AV_SAMPLE_FMT_FLTP;
  par->codec_type = AVMEDIA_TYPE_AUDIO;
  par->codec_id = codec_id;
  par->bit_rate = 64000;
  par->sample_rate = sampleRate;
  par->channels = channels;
  av_channel_layout_default(&par->ch_layout, par->channels);
  switch (par->codec_id) {
    case AV_CODEC_ID_AAC:  // AudioSpecificConfig 48000-2
      par->profile = 1;    // 0:main 1: LC 2: SSR
      // par->extradata_size = 2;
      // par->extradata      = (uint8_t *)av_malloc(par->extradata_size +
      // AV_INPUT_BUFFER_PADDING_SIZE);
      // par->extradata[0]   = 0x13;
      // par->extradata[1]   = 0x08;
      ret = av_dict_set(&options, "flvflags", "aac_seq_header_detect", 0);
      if (ret < 0) {
        tylog("av dict set flvflags ret=%d[%s]", ret, av_err2string(ret));

        return ret;
      }
      // av_dict_set_int(&options, "max_interleave_delta",
      // StreamingAgent::Get(5000000, "av_stream_out", "max_interleave_delta"),
      // 0);
      break;
    case AV_CODEC_ID_OPUS:  // OpusHead 48000-2
      par->extradata_size = 19;
      par->extradata = (uint8_t*)av_malloc(par->extradata_size +
                                           AV_INPUT_BUFFER_PADDING_SIZE);
      par->extradata[0] = 'O';
      par->extradata[1] = 'p';
      par->extradata[2] = 'u';
      par->extradata[3] = 's';
      par->extradata[4] = 'H';
      par->extradata[5] = 'e';
      par->extradata[6] = 'a';
      par->extradata[7] = 'd';
      // Version
      par->extradata[8] = 1;
      // Channel Count
      par->extradata[9] = 2;
      // Pre-skip
      par->extradata[10] = 0x38;
      par->extradata[11] = 0x1;
      // Input Sample Rate (Hz)
      par->extradata[12] = 0x80;
      par->extradata[13] = 0xbb;
      par->extradata[14] = 0;
      par->extradata[15] = 0;
      // Output Gain (Q7.8 in dB)
      par->extradata[16] = 0;
      par->extradata[17] = 0;
      // Mapping Family
      par->extradata[18] = 0;
      break;

    default:
      // should not assert
      assert(!"now not support");
  }
  audioStreamIndex_ = stream->index;

  return 0;
}

int SrtHandler::AddVideoStream_(uint32_t width, uint32_t height,
                                AVDictionary*&) {
  const enum AVCodecID codec_id = AV_CODEC_ID_H264;
  AVStream* stream = avformat_new_stream(this->formatContext_, NULL);
  if (nullptr == stream) {
    tylog("Cannot add video stream");

    return -1;
  }

  stream->time_base = AVRational{1, 90000};
  AVCodecParameters* par = stream->codecpar;
  par->codec_type = AVMEDIA_TYPE_VIDEO;
  par->codec_id = codec_id;
  par->width = width;
  par->height = height;
  videooStreamIndex_ = stream->index;

  // no par->extradata (for init decoder)

  return 0;
}

int SrtHandler::SendAudioFrame(const std::vector<char>& audioFrame,
                               uint64_t frameMs) {
  return SendFrame_(audioFrame, frameMs, true);
}

int SrtHandler::SendVideoFrame(const std::vector<char>& videoFrame,
                               uint64_t frameMs) {
  return SendFrame_(videoFrame, frameMs, false);
}

// last param should be replaced with AVStream to get its index
int SrtHandler::SendFrame_(const std::vector<char>& frame, uint64_t frameMs,
                           bool bAudio) {
  int ret = 0;

  AVPacket* pkt = av_packet_alloc();
  if (nullptr == pkt) {
    tylog("alloc pkt null, no memory");
    assert(!"allocFail");  // tmp

    return -1;
  }

  pkt->data = reinterpret_cast<uint8_t*>(const_cast<char*>(frame.data()));
  pkt->size = frame.size();
  pkt->dts = frameMs * 90;  // TS audio also 90kHz?
  pkt->pts = frameMs * 90;
  // pkt->pos = -1;
  // pkt->duration = 0;
  pkt->stream_index = bAudio ? audioStreamIndex_ : videooStreamIndex_;
  pkt->time_base = AVRational{1, bAudio ? 90000 : 90000};

  tylog("stmIdx=%d, timebase %d / %d, now_ms=%s, frameMs=%lu, pts=%ld.",
        pkt->stream_index,
        formatContext_->streams[pkt->stream_index]->time_base.num,
        formatContext_->streams[pkt->stream_index]->time_base.den,
        tylib::MilliSecondToLocalTimeString(g_now_ms).data(), frameMs,
        pkt->pts);

  ret = av_interleaved_write_frame(formatContext_, pkt);
  if (ret < 0) {
    tylog("interleaved_write_frame ret=%d[%s].", ret, av_err2string(ret));

    av_packet_free(&pkt);

    return ret;
  }

  av_packet_free(&pkt);

  return 0;
}
}  // namespace tywebrtc
