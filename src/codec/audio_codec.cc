#include "codec/audio_codec.h"

#include <cassert>

#include "log/log.h"

static const AVCodec *srs_find_decoder_by_id(SrsAudioCodecId id) {
  if (id == SrsAudioCodecIdAAC) {
    return avcodec_find_decoder_by_name("aac");
  } else if (id == SrsAudioCodecIdMP3) {
    return avcodec_find_decoder_by_name("mp3");
  } else if (id == SrsAudioCodecIdOpus) {
#ifdef SRS_FFMPEG_OPUS
    // TODO: FIXME: Note that the audio might be corrupted, see
    // https://github.com/ossrs/srs/issues/3140
    return avcodec_find_decoder_by_name("opus");
#else
    return avcodec_find_decoder_by_name("libopus");
#endif
  }
  return NULL;
}

static const AVCodec *srs_find_encoder_by_id(SrsAudioCodecId id) {
  if (id == SrsAudioCodecIdAAC) {
    return avcodec_find_encoder_by_name("aac");
  } else if (id == SrsAudioCodecIdOpus) {
#ifdef SRS_FFMPEG_OPUS
    // TODO: FIXME: Note that the audio might be corrupted, see
    // https://github.com/ossrs/srs/issues/3140
    return avcodec_find_encoder_by_name("opus");
#else
    return avcodec_find_encoder_by_name("libopus");
#endif
  }
  return NULL;
}

class SrsFFmpegLogHelper {
 public:
  SrsFFmpegLogHelper() {
    av_log_set_callback(ffmpeg_log_callback);
    av_log_set_level(AV_LOG_TRACE);
  }

  static void ffmpeg_log_callback(void *, int level, const char *fmt,
                                  va_list vl) {
    static char buf[4096] = {0};
    int nbytes = vsnprintf(buf, sizeof(buf), fmt, vl);
    if (nbytes > 0 && nbytes < (int)sizeof(buf)) {
      // Srs log is always start with new line, replcae '\n' to '\0', make log
      // easy to read.
      if (buf[nbytes - 1] == '\n') {
        buf[nbytes - 1] = '\0';
      }
      switch (level) {
        case AV_LOG_PANIC:
        case AV_LOG_FATAL:
        case AV_LOG_ERROR:
          tylog("%s", buf);
          break;
        case AV_LOG_WARNING:
          tylog("%s", buf);
          break;
        case AV_LOG_INFO:
          tylog("%s", buf);
          break;
        case AV_LOG_VERBOSE:
        case AV_LOG_DEBUG:
        case AV_LOG_TRACE:
        default:
          tylog("%s", buf);
          break;
      }
    }
  }
};

// Register FFmpeg log callback funciton.
SrsFFmpegLogHelper _srs_ffmpeg_log_helper;

SrsAudioTranscoder::SrsAudioTranscoder() {
  dec_ = NULL;
  dec_frame_ = NULL;
  dec_packet_ = NULL;
  enc_ = NULL;
  enc_frame_ = NULL;
  enc_packet_ = NULL;
  swr_ = NULL;
  swr_data_ = NULL;
  fifo_ = NULL;
  new_pkt_pts_ = AV_NOPTS_VALUE;
  next_out_pts_ = AV_NOPTS_VALUE;
}

SrsAudioTranscoder::~SrsAudioTranscoder() {
  if (dec_) {
    avcodec_free_context(&dec_);
  }

  if (dec_frame_) {
    av_frame_free(&dec_frame_);
  }

  if (dec_packet_) {
    av_packet_free(&dec_packet_);
  }

  if (swr_) {
    swr_free(&swr_);
  }

  free_swr_samples();

  if (enc_) {
    avcodec_free_context(&enc_);
  }

  if (enc_frame_) {
    av_frame_free(&enc_frame_);
  }

  if (enc_packet_) {
    av_packet_free(&enc_packet_);
  }

  if (fifo_) {
    av_audio_fifo_free(fifo_);
    fifo_ = NULL;
  }
}

int SrsAudioTranscoder::initialize(SrsAudioCodecId src_codec,
                                   SrsAudioCodecId dst_codec, int dst_channels,
                                   int dst_samplerate, int dst_bit_rate) {
  int err = 0;

  if ((err = init_dec(src_codec)) != 0) {
    tylog("dec init codec:%d", src_codec);
    return -1;
  }

  if ((err = init_enc(dst_codec, dst_channels, dst_samplerate, dst_bit_rate)) !=
      0) {
    tylog("enc init codec:%d, channels:%d, samplerate:%d, bitrate:%d",
          dst_codec, dst_channels, dst_samplerate, dst_bit_rate);
    return -1;
  }

  if ((err = init_fifo()) != 0) {
    tylog("fifo init");
    return -1;
  }

  return err;
}

int SrsAudioTranscoder::transcode(const SrsAudioFrame &in_pkt,
                                  std::vector<SrsAudioFrame> &out_pkts) {
  int err = 0;

  if ((err = decode_and_resample(in_pkt)) != 0) {
    tylog("decode and resample");
    return -1;
  }

  if ((err = encode(out_pkts)) != 0) {
    tylog("encode");
    return -1;
  }

  return err;
}

void SrsAudioTranscoder::aac_codec_header(uint8_t **data, int *len) {
  // srs_assert(dst_codec == SrsAudioCodecIdAAC);
  *len = enc_->extradata_size;
  *data = enc_->extradata;
}

int SrsAudioTranscoder::init_dec(SrsAudioCodecId src_codec) {
  const AVCodec *codec = srs_find_decoder_by_id(src_codec);
  if (!codec) {
    tylog("Codec not found by %d", src_codec);
    return -1;
  }

  dec_ = avcodec_alloc_context3(codec);
  if (!dec_) {
    tylog("Could not allocate audio codec context");
    return -1;
  }

  if (avcodec_open2(dec_, codec, NULL) < 0) {
    tylog("Could not open codec");
    return -1;
  }

  dec_->channel_layout = av_get_default_channel_layout(dec_->channels);

  dec_frame_ = av_frame_alloc();
  if (!dec_frame_) {
    tylog("Could not allocate audio decode out frame");
    return -1;
  }

  dec_packet_ = av_packet_alloc();
  if (!dec_packet_) {
    tylog("Could not allocate audio decode in packet");
    return -1;
  }

  new_pkt_pts_ = AV_NOPTS_VALUE;
  return 0;
}

int SrsAudioTranscoder::init_enc(SrsAudioCodecId dst_codec, int dst_channels,
                                 int dst_samplerate, int dst_bit_rate) {
  const AVCodec *codec = srs_find_encoder_by_id(dst_codec);
  if (!codec) {
    tylog("Codec not found by %d", dst_codec);
    return -1;
  }

  enc_ = avcodec_alloc_context3(codec);
  if (!enc_) {
    tylog("Could not allocate audio codec context %d", dst_codec);
    return -1;
  }

  enc_->sample_rate = dst_samplerate;
  enc_->channels = dst_channels;
  enc_->channel_layout = av_get_default_channel_layout(dst_channels);
  enc_->bit_rate = dst_bit_rate;
  enc_->sample_fmt = codec->sample_fmts[0];
  enc_->time_base.num = 1;
  enc_->time_base.den = 1000;  // {1, 1000}
  if (dst_codec == SrsAudioCodecIdOpus) {
    // TODO: for more level setting
    enc_->compression_level = 1;
    enc_->strict_std_compliance = FF_COMPLIANCE_EXPERIMENTAL;
  } else if (dst_codec == SrsAudioCodecIdAAC) {
    enc_->strict_std_compliance = FF_COMPLIANCE_EXPERIMENTAL;
  }

  // TODO: FIXME: Show detail error.
  if (avcodec_open2(enc_, codec, NULL) < 0) {
    tylog("Could not open codec");
    return -1;
  }

  enc_frame_ = av_frame_alloc();
  if (!enc_frame_) {
    tylog("Could not allocate audio encode in frame");
    return -1;
  }

  enc_frame_->format = enc_->sample_fmt;
  enc_frame_->nb_samples = enc_->frame_size;
  enc_frame_->channel_layout = enc_->channel_layout;

  if (av_frame_get_buffer(enc_frame_, 0) < 0) {
    tylog("Could not get audio frame buffer");
    return -1;
  }

  enc_packet_ = av_packet_alloc();
  if (!enc_packet_) {
    tylog("Could not allocate audio encode out packet");
    return -1;
  }

  next_out_pts_ = AV_NOPTS_VALUE;
  return 0;
}

int SrsAudioTranscoder::init_swr(AVCodecContext *decoder) {
  swr_ = swr_alloc_set_opts(NULL, enc_->channel_layout, enc_->sample_fmt,
                            enc_->sample_rate, decoder->channel_layout,
                            decoder->sample_fmt, decoder->sample_rate, 0, NULL);
  if (!swr_) {
    tylog("alloc swr");
    return -1;
  }

  int error;
  char err_buf[AV_ERROR_MAX_STRING_SIZE] = {0};
  if ((error = swr_init(swr_)) < 0) {
    tylog("open swr(%d:%s)", error,
          av_make_error_string(err_buf, AV_ERROR_MAX_STRING_SIZE, error));

    return error;
  }

  /* Allocate as many pointers as there are audio channels.
  * Each pointer will later point to the audio samples of the corresponding
  * channels (although it may be NULL for interleaved formats).
  */
  if (!(swr_data_ = (uint8_t **)calloc(enc_->channels, sizeof(*swr_data_)))) {
    tylog("alloc swr buffer");
    return -1;
  }

  /* Allocate memory for the samples of all channels in one consecutive
  * block for convenience. */
  if ((error = av_samples_alloc(swr_data_, NULL, enc_->channels,
                                enc_->frame_size, enc_->sample_fmt, 0)) < 0) {
    tylog("alloc swr buffer(%d:%s)", error,

          av_make_error_string(err_buf, AV_ERROR_MAX_STRING_SIZE, error));

    return -1;
  }

  return 0;
}

int SrsAudioTranscoder::init_fifo() {
  if (!(fifo_ = av_audio_fifo_alloc(enc_->sample_fmt, enc_->channels, 1))) {
    tylog("Could not allocate FIFO");
    return -1;
  }
  return 0;
}

int SrsAudioTranscoder::decode_and_resample(const SrsAudioFrame &pkt) {
  int err = 0;

  dec_packet_->data =
      reinterpret_cast<uint8_t *>(const_cast<char *>(pkt.s.data()));
  dec_packet_->size = pkt.s.size();

  // Ignore empty packet, see
  // https://github.com/ossrs/srs/pull/2757#discussion_r759797651
  if (!dec_packet_->data || !dec_packet_->size) {
    return err;
  }

  char err_buf[AV_ERROR_MAX_STRING_SIZE] = {0};
  int error = avcodec_send_packet(dec_, dec_packet_);
  if (error < 0) {
    tylog("submit to dec(%d,%s)", error,
          av_make_error_string(err_buf, AV_ERROR_MAX_STRING_SIZE, error));
    return -1;
  }

  new_pkt_pts_ = pkt.ts_ms;
  while (error >= 0) {
    error = avcodec_receive_frame(dec_, dec_frame_);
    if (error == AVERROR(EAGAIN) || error == AVERROR_EOF) {
      return err;
    } else if (error < 0) {
      tylog("Error during decoding(%d,%s)", error,
            av_make_error_string(err_buf, AV_ERROR_MAX_STRING_SIZE, error));

      return -1;
    }

    // Decoder is OK now, try to init swr if not initialized.
    if (!swr_ && (err = init_swr(dec_)) != 0) {
      tylog("resample init");
      return -1;
    }

    int in_samples = dec_frame_->nb_samples;
    const uint8_t **in_data = (const uint8_t **)dec_frame_->extended_data;
    do {
      /* Convert the samples using the resampler. */
      int frame_size =
          swr_convert(swr_, swr_data_, enc_->frame_size, in_data, in_samples);
      if ((error = frame_size) < 0) {
        tylog("Could not convert input samples(%d,%s)", error,
              av_make_error_string(err_buf, AV_ERROR_MAX_STRING_SIZE, error));
        return -1;
      }

      in_data = NULL;
      in_samples = 0;
      if ((err = add_samples_to_fifo(swr_data_, frame_size)) != 0) {
        tylog("write samples");
        return err;
      }
    } while (swr_get_out_samples(swr_, in_samples) >= enc_->frame_size);
  }

  return err;
}

int SrsAudioTranscoder::encode(std::vector<SrsAudioFrame> &pkts) {
  char err_buf[AV_ERROR_MAX_STRING_SIZE] = {0};

  if (next_out_pts_ == AV_NOPTS_VALUE) {
    next_out_pts_ = new_pkt_pts_;
  } else {
    int64_t diff = llabs(new_pkt_pts_ - next_out_pts_);
    if (diff > 1000) {
      tylog("time diff to large=%ld, next out=%ld, new pkt=%ld, set to new pkt",
            diff, next_out_pts_, new_pkt_pts_);
      next_out_pts_ = new_pkt_pts_;
    }
  }

  int frame_cnt = 0;
  while (av_audio_fifo_size(fifo_) >= enc_->frame_size) {
    /* Read as many samples from the FIFO buffer as required to fill the frame.
    * The samples are stored in the frame temporarily. */
    if (av_audio_fifo_read(fifo_, (void **)enc_frame_->data, enc_->frame_size) <
        enc_->frame_size) {
      tylog("Could not read data from FIFO");
      return -1;
    }
    /* send the frame for encoding */
    enc_frame_->pts = next_out_pts_ + av_rescale(enc_->frame_size * frame_cnt,
                                                 1000, enc_->sample_rate);
    ++frame_cnt;
    int error = avcodec_send_frame(enc_, enc_frame_);
    if (error < 0) {
      tylog("Error sending the frame to the encoder(%d,%s)", error,
            av_make_error_string(err_buf, AV_ERROR_MAX_STRING_SIZE, error));

      return -1;
    }

    av_init_packet(enc_packet_);
    enc_packet_->data = NULL;
    enc_packet_->size = 0;
    /* read all the available output packets (in general there may be any
    * number of them */
    while (error >= 0) {
      error = avcodec_receive_packet(enc_, enc_packet_);
      if (error == AVERROR(EAGAIN) || error == AVERROR_EOF) {
        break;
      } else if (error < 0) {
        tylog("Error during decoding(%d,%s)", error,
              av_make_error_string(err_buf, AV_ERROR_MAX_STRING_SIZE, error));

        return -1;
      }

      SrsAudioFrame out_frame;
      out_frame.s.assign(enc_packet_->data,
                         enc_packet_->data + enc_packet_->size);
      tylog("enc aac packet dts=%lu, pts=%lu. size=%d.", enc_packet_->dts,
            enc_packet_->pts, enc_packet_->size);
      assert(enc_packet_->dts == enc_packet_->pts);
      out_frame.ts_ms = enc_packet_->dts;

      pkts.push_back(out_frame);
    }
  }

  next_out_pts_ +=
      av_rescale(enc_->frame_size * frame_cnt, 1000, enc_->sample_rate);

  return 0;
}

int SrsAudioTranscoder::add_samples_to_fifo(uint8_t **samples, int frame_size) {
  char err_buf[AV_ERROR_MAX_STRING_SIZE] = {0};

  int error;

  /* Make the FIFO as large as it needs to be to hold both,
   * the old and the new samples. */
  if ((error = av_audio_fifo_realloc(
           fifo_, av_audio_fifo_size(fifo_) + frame_size)) < 0) {
    tylog("Could not reallocate FIFO(%d,%s)", error,
          av_make_error_string(err_buf, AV_ERROR_MAX_STRING_SIZE, error));
    return -1;
  }

  /* Store the new samples in the FIFO buffer. */
  if ((error = av_audio_fifo_write(fifo_, (void **)samples, frame_size)) <
      frame_size) {
    tylog("Could not write data to FIFO(%d,%s)", error,
          av_make_error_string(err_buf, AV_ERROR_MAX_STRING_SIZE, error));

    return -1;
  }

  return 0;
}

void SrsAudioTranscoder::free_swr_samples() {
  if (swr_data_) {
    av_freep(&swr_data_[0]);
    free(swr_data_);
    swr_data_ = NULL;
  }
}
