#ifndef RTP_PACK_UNPACK_RTP_TO_H264_H_
#define RTP_PACK_UNPACK_RTP_TO_H264_H_

#include <cassert>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include "tylib/string/any_to_string.h"

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
  MediaData() = default;

  MediaData(MediaType media_type, std::string &&data, uint32_t payload_type,
            uint32_t rtp_timestamp, uint64_t timestamp,

            int composition_timestamp, VideoRotation rotate_angle)
      : media_type_(media_type),
        data_(data),
        payload_type_(payload_type),
        rtp_timestamp_(rtp_timestamp),
        timestamp_(timestamp),
        composition_timestamp_(composition_timestamp),
        rotate_angle_(rotate_angle) {}

  // copy constructor should use move

  // should be private
  MediaType media_type_ = kMediaMax;
  std::string data_;
  uint32_t payload_type_ = 0;
  uint32_t rtp_timestamp_ = 0;
  uint64_t timestamp_ = 0;
  int composition_timestamp_ = 0;
  VideoRotation rotate_angle_ = kVideoRotation0;
};

struct FrameItem {
  std::string rawData;
  uint8_t payload_type = 0;
  uint32_t timestamp = 0;
  int cts = 0;

  FrameItem(std::string &&rawData, uint8_t payload_type, uint32_t timestamp,
            int cts)
      : rawData(rawData),
        payload_type(payload_type),
        timestamp(timestamp),
        cts(cts) {}

  std::string ToString() const {
    return tylib::format_string(
        "{rawData.size=%zu, payload_type=%d, ts=%d, cts=%d}", rawData.size(),
        payload_type, timestamp, cts);
  }
};

struct FrameBuffer {
  std::vector<FrameItem> frames;

  FrameItem &PushNewFrame(const RtpHeader &h);
  std::vector<MediaData> PopFrames();

  std::string ToString() const {
    return tylib::format_string("{frames=%s}",
                                tylib::AnyToString(frames).data());
  }
};

struct VideoUnPackParam {
  uint32_t pre_fu_valid = 0;
  uint16_t cur_rtp_seq_no = 0xFFFF;
  uint32_t pre_seq_num = 0xFFFFFFFF;
  std::string raw_stm_buff;
  int32_t raw_stm_size = 0;
  uint32_t cur_frame_ts = 0;
  VideoRotation rotate_angle = kVideoRotation0;
  uint8_t cam_type = 0;
  uint16_t max_play_out_delay_ms = 0;
  uint16_t min_play_out_delay_ms = 0;
  std::string pps;
  std::string sps;
  uint8_t wait_i_frame_flag = 0;
  uint32_t ssrc = 0;
  uint32_t payload_type = 0;
  uint8_t drop_flag = 0;
};

class H264Unpacketizer {
 public:
  int Unpacketize(const std::vector<char> &vBufReceive,
                  std::vector<MediaData> *o_mediaList);
  int DumpRawStream(const std::string &rawStream, uint32_t ssrc);

 private:
  int UpdataSps(const char *data, size_t len);
  int UpdataPps(const char *data, size_t len);

  int ParseFuaNalu(const std::vector<char> &vBufReceive);
  int ParseStapAStartOffsets(const char *nalu_ptr, int length_remaining,
                             std::vector<size_t> *offsets);
  int ParseStapAOrSingleNalu(const std::vector<char> &vBufReceive);

 private:
  // key is ssrc
  FrameBuffer frame_buffer_;
  VideoUnPackParam unpack_params_;
  bool started_ = false;
  uint16_t last_seq_num_ = 0;
  FILE *rtp_2_h264_file_ = nullptr;
};

#endif  // RTP_PACK_UNPACK_RTP_TO_H264_H_