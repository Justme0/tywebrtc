#ifndef RTP_PACK_UNPACK_RTP_TO_H264_H_
#define RTP_PACK_UNPACK_RTP_TO_H264_H_

#include <vector>

#include "tylib/string/any_to_string.h"

#include "rtp/rtp_parser.h"

#define _DUMP_RTP_ 1

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