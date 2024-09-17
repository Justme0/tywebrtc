// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// from SRS

#ifndef SRC_CODEC_AUDIO_CODEC_H_
#define SRC_CODEC_AUDIO_CODEC_H_

#include <string>
#include <vector>

#include "tylib/string/format_string.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/channel_layout.h>
#include <libavutil/frame.h>
#include <libavutil/mem.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}

namespace tywebrtc {

/**
 * The audio codec id.
 * @doc video_file_format_spec_v10_1.pdf, page 76, E.4.2 Audio Tags
 * SoundFormat UB [4]
 * Format of SoundData. The following values are defined:
 *     0 = Linear PCM, platform endian
 *     1 = ADPCM
 *     2 = MP3
 *     3 = Linear PCM, little endian
 *     4 = Nellymoser 16 kHz mono
 *     5 = Nellymoser 8 kHz mono
 *     6 = Nellymoser
 *     7 = G.711 A-law logarithmic PCM
 *     8 = G.711 mu-law logarithmic PCM
 *     9 = reserved
 *     10 = AAC
 *     11 = Speex
 *     14 = MP3 8 kHz
 *     15 = Device-specific sound
 * Formats 7, 8, 14, and 15 are reserved.
 * AAC is supported in Flash Player 9,0,115,0 and higher.
 * Speex is supported in Flash Player 10 and higher.
 */
enum SrsAudioCodecId {
  // set to the max value to reserved, for array map.
  SrsAudioCodecIdReserved1 = 16,
  SrsAudioCodecIdForbidden = 16,

  // for user to disable audio, for example, use pure video hls.
  SrsAudioCodecIdDisabled = 17,

  SrsAudioCodecIdLinearPCMPlatformEndian = 0,
  SrsAudioCodecIdADPCM = 1,
  SrsAudioCodecIdMP3 = 2,
  SrsAudioCodecIdLinearPCMLittleEndian = 3,
  SrsAudioCodecIdNellymoser16kHzMono = 4,
  SrsAudioCodecIdNellymoser8kHzMono = 5,
  SrsAudioCodecIdNellymoser = 6,
  SrsAudioCodecIdReservedG711AlawLogarithmicPCM = 7,
  SrsAudioCodecIdReservedG711MuLawLogarithmicPCM = 8,
  SrsAudioCodecIdReserved = 9,
  SrsAudioCodecIdAAC = 10,
  SrsAudioCodecIdSpeex = 11,
  // For FLV, it's undefined, we define it as Opus for WebRTC.
  SrsAudioCodecIdOpus = 13,
  SrsAudioCodecIdReservedMP3_8kHz = 14,
  SrsAudioCodecIdReservedDeviceSpecificSound = 15,
};

struct SrsAudioFrame {
  std::string s;
  int64_t ts_ms = 0;

  std::string ToString() const {
    return tylib::format_string("{size=%zu, tsMs=%" PRId64 " }", s.size(),
                                ts_ms);
  }
};

class SrsAudioTranscoder {
 public:
  SrsAudioTranscoder();
  ~SrsAudioTranscoder();

  SrsAudioTranscoder(const SrsAudioTranscoder &) = delete;
  SrsAudioTranscoder &operator=(const SrsAudioTranscoder &) = delete;

  // Initialize the transcoder, transcode from codec as to codec.
  // The channels specifies the number of output channels for encoder, for
  // example, 2.
  // The sample_rate specifies the sample rate of encoder, for example, 48000.
  // The bit_rate specifies the bitrate of encoder, for example, 48000.
  int initialize(SrsAudioCodecId from, SrsAudioCodecId to, int channels,
                 int sample_rate, int bit_rate);

  // Transcode the input audio frame in, as output audio frames outs.
  int transcode(const SrsAudioFrame &in, std::vector<SrsAudioFrame> &outs);

  // Get the aac codec header, for example, FLV sequence header.
  // @remark User should never free the data, it's managed by this transcoder.
  void aac_codec_header(uint8_t **data, int *len);

 private:
  int init_dec(SrsAudioCodecId from);
  int init_enc(SrsAudioCodecId to, int channels, int samplerate, int bit_rate);
  int init_swr(AVCodecContext *decoder);
  int init_fifo();

  int decode_and_resample(const SrsAudioFrame &pkt);
  int encode(std::vector<SrsAudioFrame> &pkts);

  int add_samples_to_fifo(uint8_t **samples, int frame_size);
  void free_swr_samples();

 private:
  AVCodecContext *dec_;
  AVFrame *dec_frame_;
  AVPacket *dec_packet_;

  AVCodecContext *enc_;
  AVFrame *enc_frame_;
  AVPacket *enc_packet_;

  SwrContext *swr_;
  // buffer for swr out put
  uint8_t **swr_data_;
  AVAudioFifo *fifo_;

  int64_t new_pkt_pts_;
  int64_t next_out_pts_;
};

}  // namespace tywebrtc

#endif  // SRC_CODEC_AUDIO_CODEC_H_
