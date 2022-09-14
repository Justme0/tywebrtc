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

// Nalu type定义，根据H264协议标准定义
// https://github.com/wireshark/wireshark/blob/master/epan/dissectors/packet-h264.c#L302
enum enVideoH264NaluType {
  kVideoNakuUnspecific = 0,
  kVideoNaluSlice = 1,
  kVideoNaluDpa = 2,
  kVideoNaluDpb = 3,
  kVideoNaluDpc = 4,
  kVideoNaluIdr = 5,
  kVideoNaluSei = 6,
  kVideoNaluSps = 7,
  kVideoNaluPps = 8,
  kVideoNaluUnitDelimiterRbsp = 9,
  kVideoNaluUnitEoseq = 10,
  kVideoNaluUnitEostm = 11,
  kVideoNaluUnitFilter = 12,
  kVideoNaluUnitSpsExtn = 13,
  kH264StapA = 24,
  kH264FuA = 28,
};

inline std::string enVideoH264NaluTypeToString(enVideoH264NaluType v) {
  switch (v) {
    case kVideoNakuUnspecific:
      return "Undefined";
    case kVideoNaluSlice:
      return "NAL unit - Coded slice of a non-IDR picture";
    case kVideoNaluDpa:
      return "NAL unit - Coded slice data partition A";
    case kVideoNaluDpb:
      return "NAL unit - Coded slice data partition B";
    case kVideoNaluDpc:
      return "NAL unit - Coded slice data partition C";
    case kVideoNaluIdr:
      return "NAL unit - Coded slice of an IDR picture";
    case kVideoNaluSei:
      return "NAL unit - Supplemental enhancement information (SEI)";
    case kVideoNaluSps:
      return "NAL unit - Sequence parameter set";
    case kVideoNaluPps:
      return "NAL unit - Picture parameter set";
    case kVideoNaluUnitDelimiterRbsp:
      return "NAL unit - Access unit delimiter";
    case kVideoNaluUnitEoseq:
      return "NAL unit - End of sequence";
    case kVideoNaluUnitEostm:
      return "NAL unit - End of stream";
    case kVideoNaluUnitFilter:
      return "NAL unit - Filler data";
    case kVideoNaluUnitSpsExtn:
      return "NAL unit - Sequence parameter set extension";
      // "NAL unit - Prefix" // 14
      // "NAL unit - Subset sequence parameter set" // 15
      // "NAL unit - Reserved" // 16
      // "NAL unit - Reserved" // 17
      // "NAL unit - Reserved" // 18
      // "NAL unit - Coded slice of an auxiliary coded picture without
      // partitioning" // 19
      // "NAL unit - Coded slice extension" // 20
      // "NAL unit - Coded slice extension for depth view components" // 21
      // "NAL unit - Reserved" // 22
      // "NAL unit - Reserved" // 23
    case kH264StapA:
      return "Single-time aggregation packet A (STAP-A)";
      //  "Single-time aggregation packet B (STAP-B)" // 25
      // "Multi-time aggregation packet 16 (MTAP16)" // 26
      // "Multi-time aggregation packet 24 (MTAP24)" // 27
    case kH264FuA:
      return "Fragmentation unit A (FU-A)";
    // "Fragmentation unit B (FU-B)" // 29
    // "NAL unit - Payload Content Scalability Information (PACSI)" // 30
    // "NAL unit - Extended NAL Header" // 31
    default:
      return tylib::format_string("Unknown[%d]", v);
  }
};

// OPT: can cancel the struct ?
class MediaData {
 public:
  MediaData()
      : media_type_(kMediaMax),
        data_(NULL),
        len_(0),
        timestamp_(0),
        audio_frame_len_(0),
        rotate_angle_(kVideoRotation0),
        payload_type_(0),
        rtp_timestamp_(0) {}

  MediaData(MediaType media_type, char *data, int32_t len, uint64_t timestamp,
            uint32_t payload_type, uint32_t audio_frame_len = 0,
            uint32_t rtp_timestamp = 0,
            VideoRotation rotate_angle = kVideoRotation0) {
    media_type_ = media_type;
    data_ = data;
    len_ = len;
    timestamp_ = timestamp;
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
  uint32_t audio_frame_len_;
  VideoRotation rotate_angle_;
  uint32_t payload_type_;
  uint32_t rtp_timestamp_;
  int composition_timestamp_ = 0;
};

class H264Unpacketizer {
 public:
  H264Unpacketizer();
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
};

#endif  // RTP_CODEC_PARSER_RTP2H264_H_