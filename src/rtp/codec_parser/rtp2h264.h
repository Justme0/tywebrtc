#ifndef RTP_CODEC_PARSER_RTP2H264_H_
#define RTP_CODEC_PARSER_RTP2H264_H_

#include <cassert>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include "rtp/rtp_parser.h"

#define _DUMP_RTP_ 1

/* Nalu type定义，根据H264协议标准定义 */
enum enVideoH264NaluType {
  kVideoNakuUnspecific = 0,
  kVideoNaluSlice,
  kVideoNaluDpa,
  kVideoNaluDpb,
  kVideoNaluDpc,
  kVideoNaluIdr,
  kVideoNaluSei,
  kVideoNaluSps,
  kVideoNaluPps,
  kVideoNaluUnitDelimiterRbsp,
  kVideoNaluUnitEoseq,
  kVideoNaluUnitEostm,
  kVideoNaluUnitSpsExtn
};

// OPT: can cancel the struct ?
class MediaData {
 public:
  MediaData()
      : media_type_(kMediaMax),
        data_(NULL),
        len_(0),
        timestamp_(0),
        ssrc_(0),
        audio_frame_len_(0),
        rotate_angle_(kVideoRotation0),
        payload_type_(0),
        rtp_timestamp_(0) {}

  MediaData(MediaType media_type, char *data, int32_t len, uint64_t timestamp,
            uint32_t ssrc, uint32_t payload_type, uint32_t audio_frame_len = 0,
            uint32_t rtp_timestamp = 0,
            VideoRotation rotate_angle = kVideoRotation0) {
    media_type_ = media_type;
    data_ = data;
    len_ = len;
    timestamp_ = timestamp;
    ssrc_ = ssrc;
    rotate_angle_ = rotate_angle;
    payload_type_ = payload_type;
    audio_frame_len_ = audio_frame_len;
    rtp_timestamp_ = rtp_timestamp;
  }

  // should be private
  MediaType media_type_;
  char *data_;
  int32_t len_;
  uint64_t timestamp_;
  uint32_t ssrc_;
  uint32_t audio_frame_len_;
  VideoRotation rotate_angle_;
  uint32_t payload_type_;
  uint32_t rtp_timestamp_;
  int composition_timestamp_ = 0;
};

class H264Unpacketizer {
 public:
  explicit H264Unpacketizer(uint32_t ssrc);
  ~H264Unpacketizer();  // must refactor! remove it

  std::vector<std::unique_ptr<MediaData>> Unpacketize(
      const std::vector<char> &vBufReceive);

 private:
  void Init();

  void UpdataSps(const char *data, size_t len);
  void UpdataPps(const char *data, size_t len);

  int ParseStapAStartOffsets(const char *nalu_ptr, size_t length_remaining,
                             std::vector<size_t> *offsets);
  int ParseStapAOrSingleNalu(const std::vector<char> &vBufReceive);
  int ParseFuaNalu(const std::vector<char> &vBufReceive);

 private:
  struct FrameBuffer {
    struct SliceInfo {
      size_t length;
      uint32_t ssrc;
      uint8_t payload_type;
      uint32_t timestamp;
      int cts;
    };

    // element is heap pointer, TODO: use smart pointer
    std::vector<char *> frames;
    std::vector<SliceInfo> slice_info;

    int NewFrame(const RtpHeader &h);
    int PopSlices(std::vector<std::unique_ptr<MediaData>> &slices);

    std::string ToString() const {
      assert(frames.size() == slice_info.size());
      return tylib::format_string("frame num=%zu", frames.size());
    }
  };

 public:  // should private, now for debug
  FrameBuffer frame_buffer_;

 private:
  VideoUnPackParam unpack_params_;
  bool started_ = false;
  uint16_t last_seq_num_ = 0;
  std::string stream_id_;
  uint32_t ssrc_;
};

#endif  // RTP_CODEC_PARSER_RTP2H264_H_