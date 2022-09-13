
#include "rtp/codec_parser/rtp2h264.h"

#include "rtp/rtp_parser.h"

// #include "common/system_adapt.h"
// #include "codec_utility.h"
// #include "peerconnection/pc_data.h"

#include <cassert>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <vector>

// //  #include "common/tools.h"
// //  #include "rtp/rtp_utility.h"

// to move to tylib
template <typename T, T M>
inline typename std::enable_if<(M == 0), T>::type ForwardDiff(T a, T b) {
  static_assert(std::is_unsigned<T>::value,
                "Type must be an unsigned integer.");
  return b - a;
}

template <typename T>
inline T ForwardDiff(T a, T b) {
  return ForwardDiff<T, 0>(a, b);
}

// move to head file
const int kH264TypeMask = 0x1F;

// @see: https://tools.ietf.org/html/rfc6184#section-5.2
static const uint8_t kH264StapA = 24;
static const uint8_t kH264FuA = 28;

template <typename T, T M>
inline typename std::enable_if<(M == 0), bool>::type AheadOrAt(T a, T b) {
  static_assert(std::is_unsigned<T>::value,
                "Type must be an unsigned integer.");
  const T maxDist = std::numeric_limits<T>::max() / 2 + T(1);
  if (a - b == maxDist) return b < a;
  return ForwardDiff(b, a) < maxDist;
}

template <typename T>
inline bool AheadOrAt(T a, T b) {
  return AheadOrAt<T, 0>(a, b);
}

template <typename T, T M = 0>
inline bool AheadOf(T a, T b) {
  static_assert(std::is_unsigned<T>::value,
                "Type must be an unsigned integer.");
  return a != b && AheadOrAt<T, M>(a, b);
}

static const uint8_t kNalHeaderSize = 1;
static const uint8_t kFuAHeaderSize = 2;
static const uint8_t kLengthFieldSize = 2;
static const uint8_t kStapAHeaderSize = kNalHeaderSize + kLengthFieldSize;
static const uint8_t kNaluTypeSize = 1;

typedef struct NaluInfo {
  uint8_t type;
  // Offset and size are only valid for non-FuA packets.
  uint32_t offset;
  uint32_t size;
} NaluInfo;

int H264Unpacketizer::FrameBuffer::NewFrame(const RtpHeader &h) {
  // 最大RTP包的长度，单位BYTE
#define VIDEO_MAX_SLIC_BUFF_LEN (2048000)
  char *frame = new char[VIDEO_MAX_SLIC_BUFF_LEN];
  frames.emplace_back(frame);

  int cts = 0;

  for (const std::shared_ptr<Extension> &ext : h.getParsedExtensions()) {
    if (ext->extension_type != kRtpExtCompositionTime) {
      continue;
    }

    //     https://en.cppreference.com/w/cpp/language/dynamic_cast
    //     A downcast can also be performed with static_cast, which avoids the
    //     cost of the runtime check, but it's only safe if the program can
    //     guarantee (through some other logic) that the object pointed to by
    //     expression is definitely Derived.
    const CompositionTimeIdExt &exten =
        *static_cast<CompositionTimeIdExt *>(ext.get());
    cts = exten.cts;
    tylog("new frame cts %d", exten.cts);

    // NOTE: only one iterate logic
    break;
  }

  // what if cts = 0? that is not found composition time extension
  slice_info.emplace_back(FrameBuffer::SliceInfo{
      0, h.getSSRC(), h.getPayloadType(), h.getTimestamp(), cts});
  return 0;
}

int H264Unpacketizer::FrameBuffer::PopSlices(
    std::vector<std::unique_ptr<MediaData>> &slices) {
  for (size_t i = 0; i < frames.size(); i++) {
    auto &info = slice_info[i];
    std::unique_ptr<MediaData> raw(new MediaData(kMediaVideo, frames[i],
                                                 info.length, info.timestamp,
                                                 info.ssrc, info.payload_type));
    // bad assignment, should be in constructor
    raw->rtp_timestamp_ = info.timestamp;
    raw->composition_timestamp_ = info.cts;
    slices.emplace_back(std::move(raw));
  }
  frames.clear();
  slice_info.clear();
  return 0;
}

enum FuDefs { kSBit = 0x80, kEBit = 0x40, kRBit = 0x20 };
#define VIDEO_MAX_SPP_PPS_LEN (100)

int H264Unpacketizer::ParseFuaNalu(const std::vector<char> &vBufReceive) {
  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(vBufReceive.data());
  const auto &payload = vBufReceive.data() + rtpHeader.getHeaderLength();
  auto length = vBufReceive.size() - rtpHeader.getHeaderLength();
  if (length < kFuAHeaderSize) {
    return -1;
  }

  const int kFBit = 0x80;
  const int kNriMask = 0x60;
  uint8_t fnri = payload[0] & (kFBit | kNriMask);
  uint8_t original_nal_type = payload[1] & kH264TypeMask;
  bool first_fragment = ((payload[1] & kSBit) != 0);

  auto WriteStartCode = [](char *dst) -> size_t {
    dst[0] = 0;
    dst[1] = 0;
    dst[2] = 0;
    dst[3] = 1;
    return 4;
  };

  if (first_fragment) {
    int idx = (int)frame_buffer_.frames.size() - 1;
    if (idx < 0 ||
        (frame_buffer_.slice_info[idx].timestamp != rtpHeader.getTimestamp())) {
      frame_buffer_.NewFrame(rtpHeader);
      idx = (int)frame_buffer_.frames.size() - 1;
      if (original_nal_type == kVideoNaluIdr &&
          unpack_params_.sps_pkt_len != 0) {
        auto dst = frame_buffer_.frames[idx];
        auto &info = frame_buffer_.slice_info[idx];
        size_t len = WriteStartCode(dst);
        memcpy(dst + len, unpack_params_.sps, unpack_params_.sps_pkt_len);
        len += unpack_params_.sps_pkt_len;
        len += WriteStartCode(dst + len);
        memcpy(dst + len, unpack_params_.pps, unpack_params_.pps_pkt_len);
        len += unpack_params_.pps_pkt_len;
        info.length += len;
      }
    }
    auto &info = frame_buffer_.slice_info[idx];
    auto dst = frame_buffer_.frames[idx] + info.length;
    uint8_t original_nal_header = fnri | original_nal_type;
    size_t len = WriteStartCode(dst);
    dst[len] = original_nal_header;
    len += 1;
    info.length += len;
  }

  int idx = (int)frame_buffer_.frames.size() - 1;
  if (idx < 0) {
    return -2;
  }
  auto &info = frame_buffer_.slice_info[idx];
  if (info.timestamp != rtpHeader.getTimestamp()) {
    tylog("maybe lost packet, info.ts=%u, rtpHeader=%s", info.timestamp,
          rtpHeader.ToString().data());
    // tylor("Chn %" PRIu64
    //       " unpacketizer h264 fua noncontinues: seq[%u] ssrc = %u, "
    //       "pt = %u rtp_len=%d padding_len=%d extern_offset=%d!",
    //       ssrc_, header.sequence_number, header.ssrc, header.payload_type,
    //       length, header.padding_length, header.header_length -
    //       RTP_HEADER_LTH);
  }

  length -= kFuAHeaderSize;
  auto dst = frame_buffer_.frames[idx] + info.length;
  info.length += length;
  memcpy(dst, payload + kFuAHeaderSize, length);

  return 0;
}

int H264Unpacketizer::ParseStapAStartOffsets(const char *nalu_ptr,
                                             size_t length_remaining,
                                             std::vector<size_t> *offsets) {
  size_t offset = 0;
  while (length_remaining > 0) {
    // Buffer doesn't contain room for additional nalu length.
    if (length_remaining < sizeof(uint16_t)) {
      return -1;
    }
    uint16_t nalu_size = ((nalu_ptr[0] << 8) | nalu_ptr[1]);
    nalu_ptr += sizeof(uint16_t);
    length_remaining -= sizeof(uint16_t);
    if (nalu_size > length_remaining) {
      return -2;
    }
    nalu_ptr += nalu_size;
    length_remaining -= nalu_size;

    offsets->push_back(offset + kStapAHeaderSize);
    offset += kLengthFieldSize + nalu_size;
  }
  return 0;
}

int H264Unpacketizer::ParseStapAOrSingleNalu(
    const std::vector<char> &vBufReceive) {
  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(vBufReceive.data());
  const char *payload = vBufReceive.data() + rtpHeader.getHeaderLength();
  int length = vBufReceive.size() - rtpHeader.getHeaderLength();

  const char *nalu_start = payload + kNalHeaderSize;
  const size_t nalu_length = length - kNalHeaderSize;
  uint8_t nal_type = payload[0] & kH264TypeMask;
  std::vector<size_t> nalu_start_offsets;
  if (nal_type == kH264StapA) {
    // Skip the StapA header (StapA NAL type + length).
    if (length <= kStapAHeaderSize) {
      return -1;
    }

    int ret =
        ParseStapAStartOffsets(nalu_start, nalu_length, &nalu_start_offsets);
    if (ret) {
      tylog("parseStapAStartOffsets  ret=%d", ret);
      return ret;
    }

    nal_type = payload[kStapAHeaderSize] & kH264TypeMask;
  } else {
    nalu_start_offsets.push_back(0);
  }

  if (frame_buffer_.frames.empty()) {
    frame_buffer_.NewFrame(rtpHeader);
  } else {
    int idx = (int)frame_buffer_.frames.size() - 1;
    if (frame_buffer_.slice_info[idx].timestamp != rtpHeader.getTimestamp()) {
      frame_buffer_.NewFrame(rtpHeader);
    }
  }

  nalu_start_offsets.push_back(length + kLengthFieldSize);  // End offset.
  for (size_t i = 0; i < nalu_start_offsets.size() - 1; ++i) {
    size_t start_offset = nalu_start_offsets[i];
    // End offset is actually start offset for next unit, excluding length field
    // so remove that from this units length.
    size_t end_offset = nalu_start_offsets[i + 1] - kLengthFieldSize;
    if (end_offset - start_offset < kNaluTypeSize) {
      return -3;
    }

    NaluInfo nalu;
    nalu.type = payload[start_offset] & kH264TypeMask;
    nalu.offset = start_offset;
    nalu.size = end_offset - start_offset;
    // start_offset += kNaluTypeSize;

    int idx = (int)frame_buffer_.frames.size() - 1;
    char *buff = frame_buffer_.frames[idx];
    auto &info = frame_buffer_.slice_info[idx];
    auto &len = info.length;

    switch (nalu.type) {
      case kVideoNaluSps:
        // kVideoFrameKey;
        UpdataSps(payload + start_offset, nalu.size);
        break;
      case kVideoNaluPps:
        UpdataPps(payload + start_offset, nalu.size);
        break;
      case kVideoNaluIdr:
        // kVideoFrameKey;
        if (unpack_params_.sps_pkt_len != 0 &&
            unpack_params_.pps_pkt_len != 0 && 0 == len) {
          auto WriteStartCode = [](char *dst) -> size_t {
            dst[0] = 0;
            dst[1] = 0;
            dst[2] = 0;
            dst[3] = 1;
            return 4;
          };
          WriteStartCode(buff + len);
          len += 4;
          memcpy(buff + len, unpack_params_.sps, unpack_params_.sps_pkt_len);
          len += unpack_params_.sps_pkt_len;
          WriteStartCode(buff + len);
          len += 4;
          memcpy(buff + len, unpack_params_.pps, unpack_params_.pps_pkt_len);
          len += unpack_params_.pps_pkt_len;
        }
      case kVideoNaluSlice:
        break;
      // Slices below don't contain SPS or PPS ids.
      case kVideoNaluUnitDelimiterRbsp:
      case kVideoNaluUnitEoseq:
      case kVideoNaluUnitEostm:
      case kVideoNaluUnitSpsExtn:
      case kVideoNaluSei:
        break;
      case kH264StapA:
      case kH264FuA:
        delete[] buff;
        return -4;
    }

    buff[len] = 0;
    buff[len + 1] = 0;
    buff[len + 2] = 0;
    buff[len + 3] = 1;

    // 两帧间的最小时戳差值
#define VIDEO_NAL_START_CODE_LEN (4)
    len += VIDEO_NAL_START_CODE_LEN;
    memcpy(buff + len, payload + start_offset, nalu.size);
    len += nalu.size;
  }

  return 0;
}

H264Unpacketizer::H264Unpacketizer(uint32_t ssrc) : ssrc_(ssrc) {
  unpack_params_.raw_stm_buff = NULL;
  unpack_params_.pps = NULL;
  unpack_params_.sps = NULL;
  unpack_params_.drop_flag = 0;

  Init();
}

H264Unpacketizer::~H264Unpacketizer() {
  VideoUnPackParam *unpack_params = &(unpack_params_);

  /* 初始化记录参数 */
  unpack_params->raw_stm_size = 0;
  unpack_params->pre_fu_valid = 0;

  if (unpack_params_.raw_stm_buff) {
    delete[] unpack_params_.raw_stm_buff;
    unpack_params_.raw_stm_buff = NULL;
  }

  if (unpack_params_.pps) {
    delete[] unpack_params_.pps;
    unpack_params_.pps = NULL;
  }

  if (unpack_params_.sps) {
    delete[] unpack_params_.sps;
    unpack_params_.sps = NULL;
  }
}

void H264Unpacketizer::Init() {
  unpack_params_.pre_fu_valid = 0;
  unpack_params_.cur_rtp_seq_no = 0xffff;
  unpack_params_.pre_seq_num = 0xFFFFFFFF;
  unpack_params_.raw_stm_size = 0;
  unpack_params_.cur_frame_ts = 0;
  // unpack_params_.cam_type                 = kVideoFrontFacingCam;
  unpack_params_.max_play_out_delay_ms = 0;
  unpack_params_.min_play_out_delay_ms = 0;
  unpack_params_.rotate_angle = kVideoRotation0;
  unpack_params_.wait_i_frame_flag = 0;
  unpack_params_.drop_flag = 0;
  unpack_params_.pps_pkt_len = 0;
  unpack_params_.sps_pkt_len = 0;

  // memset(&status_, 0, sizeof status_);
  /*or (int32_t i = 0; i < MAX_RTP_EXT_TYPE; i++) {
      loc_id_to_rtc_ext_id_[i] = i;
      rtc_ext_id_to_loc_id_[i] = i;
  }*/

  if (NULL == unpack_params_.raw_stm_buff) {
    unpack_params_.raw_stm_buff = new uint8_t[VIDEO_MAX_SLIC_BUFF_LEN];
  }

  if (NULL == unpack_params_.pps) {
    unpack_params_.pps = new uint8_t[VIDEO_MAX_SPP_PPS_LEN];
  }

  if (NULL == unpack_params_.sps) {
    unpack_params_.sps = new uint8_t[VIDEO_MAX_SPP_PPS_LEN];
  }

  return;
}

std::vector<std::unique_ptr<MediaData>> H264Unpacketizer::Unpacketize(
    const std::vector<char> &vBufReceive) {
  int ret = 0;
  std::vector<std::unique_ptr<MediaData>> frames;

  uint32_t lost_pkt_cnt = 0;
  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(vBufReceive.data());

  if (!started_) {
    started_ = true;
    last_seq_num_ = rtpHeader.getSeqNumber();
  }

  if (AheadOf(last_seq_num_, rtpHeader.getSeqNumber())) {
    return frames;
  }

  if (last_seq_num_++ != rtpHeader.getSeqNumber()) {
    tylog("Chn %" PRIu32
          " find lost pkt(%d), last pkt(%d), have fus, buf still "
          "not find fu-end!",
          ssrc_, rtpHeader.getSeqNumber(), last_seq_num_);

    last_seq_num_ = rtpHeader.getSeqNumber() + 1;
    // return frames;
  }

  // status_.in_buf_cnt++; // just for monitor
  // VideoUnPackCheckLost(rtp_data, rtp_len, lost_pkt_cnt);

  unpack_params_.cur_rtp_seq_no =
      (unpack_params_.cur_rtp_seq_no + lost_pkt_cnt) & 0x00FFFF;

  //发现webrtc收到全padding的数据，此数据非264数据，丢弃
  if ((vBufReceive.size() <=
       getRtpPaddingLength(vBufReceive) + rtpHeader.getHeaderLength() + 1)) {
    if (vBufReceive.size() !=
        getRtpPaddingLength(vBufReceive) + rtpHeader.getHeaderLength()) {
      tylog(
          "SSRC %u receive an unknow rtp with padding, rtp=%s, paddinglen=%d, "
          "recv buf len=%zu.",
          ssrc_, rtpHeader.ToString().data(), getRtpPaddingLength(vBufReceive),
          vBufReceive.size());
    }

    return frames;
  }

  const char *payload = vBufReceive.data() + rtpHeader.getHeaderLength();
  uint8_t nal_type = payload[0] & kH264TypeMask;
  if (nal_type == kH264FuA) {
    ret = ParseFuaNalu(vBufReceive);
    if (ret) {
      tylog("parseFuaNalu ret=%d", ret);
    }
  } else {
    ret = ParseStapAOrSingleNalu(vBufReceive);
    if (ret) {
      tylog("parseStapAOrSingleNaluarseFuaNalu ret=%d", ret);
    }
  }
  if (rtpHeader.getMarker()) {
    frame_buffer_.PopSlices(frames);
  }

  // status_.recv_frame_cnt += frames.size();
  return frames;
}

void H264Unpacketizer::UpdataSps(const char *data, size_t len) {
  // status_.recv_sps_cnt++;
  if ((len < VIDEO_MAX_SPP_PPS_LEN) && (unpack_params_.sps)) {
    memcpy(unpack_params_.sps, data, len);
    unpack_params_.sps_pkt_len = len;
  }
  // status_.sps_pkt_len = len;
}

void H264Unpacketizer::UpdataPps(const char *data, size_t len) {
  if ((len < VIDEO_MAX_SPP_PPS_LEN) && (unpack_params_.pps)) {
    memcpy(unpack_params_.pps, data, len);
    unpack_params_.pps_pkt_len = len;
  }
  // status_.pps_pkt_len = len;
}
