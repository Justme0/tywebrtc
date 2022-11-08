
#include "rtp/pack_unpack/rtp_to_h264.h"

#include <cassert>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <vector>

#include "rtp/pack_unpack/pack_unpack_common.h"
#include "rtp/rtp_parser.h"

static const uint8_t kNalHeaderSize = 1;
static const uint8_t kFuAHeaderSize = 2;
static const uint8_t kLengthFieldSize = 2;
static const uint8_t kStapAHeaderSize = kNalHeaderSize + kLengthFieldSize;
static const uint8_t kNaluTypeSize = 1;

struct NaluInfo {
  enVideoH264NaluType type;
  // Offset and size are only valid for non-FuA packets.
  uint32_t offset;
  uint32_t size;

  std::string ToString() const {
    return tylib::format_string("{type=%d[%s], offset=%d, size=%d}", type,
                                enVideoH264NaluTypeToString(type).data(),
                                offset, size);
  }
};

#define VIDEO_MAX_SLIC_BUFF_LEN (2048000)

// Generate a new frame according to RTP header, and push to buffer at last
// @return the new frame (last position), for convenience
FrameItem &FrameBuffer::PushNewFrame(const RtpHeader &h) {
  int cts = 0;
  // to get cts for new frame
  for (const std::shared_ptr<Extension> &ext : h.parseExtensions()) {
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
  frames.emplace_back(std::string(), h.getPayloadType(), h.getTimestamp(), cts);

  return frames.back();
}

// @brief
// 以mediaData形式取出所有保存的frames，并清空内部所有数据，取出的数据需要有序
std::vector<MediaData> FrameBuffer::PopFrames() {
  // RVO
  std::vector<MediaData> v;
  for (FrameItem &f : frames) {
    // move raw data, frames will be cleared
    v.emplace_back(kMediaVideo, std::move(f.rawData), f.payload_type,
                   f.timestamp, f.timestamp, f.cts, kVideoRotation0);
  }
  frames.clear();

  // https://stackoverflow.com/questions/11817873/using-stdmove-when-returning-a-value-from-a-function-to-avoid-to-copy
  // OPT: avoid copy
  // MediaData should support move constructor
  return v;
}

// use constexpr?
const std::string kH264StartCodeCharArray({0, 0, 0, 1});

int H264Unpacketizer::ParseFuaNalu(const std::vector<char> &vBufReceive) {
  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(vBufReceive.data());
  char const *const payload = vBufReceive.data() + rtpHeader.getHeaderLength();
  int length = vBufReceive.size() - rtpHeader.getHeaderLength();
  if (length <= kFuAHeaderSize) {
    tylog("error: payload(include padding?) length=%d <= kFuAHeaderSize=%d",
          length, kFuAHeaderSize);
    return -1;
  }

  enVideoH264NaluType original_nal_type = static_cast<enVideoH264NaluType>(
      payload[1] & kH264TypeMask);  // taylor not payload[0] ?
  bool first_fragment = ((payload[1] & kSBit) != 0);
  tylog("original_nal_type=%s, first_fragment=%d.",
        enVideoH264NaluTypeToString(original_nal_type).data(), first_fragment);

  if (first_fragment) {
    if (frame_buffer_.frames.empty() ||
        frame_buffer_.frames.back().timestamp != rtpHeader.getTimestamp()) {
      FrameItem &frame = frame_buffer_.PushNewFrame(rtpHeader);

      // should also check pps len?
      // add SPS, PPS before I frame
      if (original_nal_type == kVideoNaluIdr) {
        if (unpack_params_.sps.empty()) {
          tylog("get IDR, but have no SPS before");
          return -2;
        }
        if (unpack_params_.pps.empty()) {
          tylog("get IDR, but have no PPS before");
          return -3;
        }

        assert(frame.rawData.empty() || !"new frame.rawData should empty");
        frame.rawData.append(kH264StartCodeCharArray)
            .append(unpack_params_.sps)
            .append(kH264StartCodeCharArray)
            .append(unpack_params_.pps);

        tylog("recv FU-A IDR frame, sps len=%zu, pps len=%zu",
              unpack_params_.sps.size(), unpack_params_.pps.size());
      }
    }
    FrameItem &frame = frame_buffer_.frames.back();

    // +---------------+
    // |0|1|2|3|4|5|6|7|
    // +-+-+-+-+-+-+-+-+
    // |F|NRI|  Type   |
    // +---------------+
    // https://segmentfault.com/a/1190000006698552?utm_source=sf-backlinks
    // The first bit of this sequence ( which is a 0 ) is the forbidden zero and
    // is used to verify if errors where encountered during the transmission of
    // the packet.
    const int kFBit = 0x80;
    // The following 2 bits ( the 11 ) are called nal_ref_idc and they indicates
    // if NAL unit is a reference field, frame or picture.
    const int kNriMask = 0x60;
    uint8_t fnri = payload[0] & (kFBit | kNriMask);  // taylor not payload[1] ?

    char original_nal_header = fnri | original_nal_type;
    frame.rawData.append(kH264StartCodeCharArray).append({original_nal_header});
  }
  if (frame_buffer_.frames.empty()) {
    tylog("frame_buffer_.frames empty, error!");
    return -4;
  }

  FrameItem &frame = frame_buffer_.frames.back();
  if (frame.timestamp != rtpHeader.getTimestamp()) {
    tylog("maybe lost packet, NOTE timestamp: buffer frame=%s, rtpHeader=%s",
          frame.ToString().data(), rtpHeader.ToString().data());
    // tylor("Chn %" PRIu64
    //       " unpacketizer h264 fua noncontinues: seq[%u] ssrc = %u, "
    //       "pt = %u rtp_len=%d padding_len=%d extern_offset=%d!",
    //       ssrc_, header.sequence_number, header.ssrc, header.payload_type,
    //       length, header.padding_length, header.header_length -
    //       RTP_HEADER_LTH);
  }

  length -= kFuAHeaderSize;
  // may have padding?
  // TODO: check buffer overflow !!!
  // FU-A no need extra start code
  frame.rawData.append(payload + kFuAHeaderSize, length);

  return 0;
}

int H264Unpacketizer::ParseStapAStartOffsets(const char *nalu_ptr,
                                             int length_remaining,
                                             std::vector<size_t> *offsets) {
  size_t offset = 0;
  while (length_remaining > 0) {
    // Buffer doesn't contain room for additional nalu length.
    const int kNaluLenSizeB = 2;
    if (length_remaining < kNaluLenSizeB) {
      tylog("err: length_remaining=%d too short(< 2 Byte)", length_remaining);

      return -1;
    }

    uint16_t nalu_size = ntohs(*reinterpret_cast<const uint16_t *>(nalu_ptr));
    tylog("nalu size=%d", nalu_size);
    nalu_ptr += sizeof(uint16_t);
    length_remaining -= sizeof(uint16_t);
    if (nalu_size > length_remaining) {
      tylog("err: nalu_size=%d > length_remaining=%d", nalu_size,
            length_remaining);

      return -2;
    }

    nalu_ptr += nalu_size;
    length_remaining -= nalu_size;

    offsets->push_back(offset + kStapAHeaderSize);
    offset += kLengthFieldSize + nalu_size;
  }

  return 0;
}

// move to tylib, use string_view
// pool performance, should re-design interface
std::string ConvertToReadableHex(const std::vector<char> &s) {
  std::string hexString;
  bool first = true;
  for (unsigned char c : s) {
    char arr[3];
    if (!first) {
      hexString.append({' '});
    } else {
      first = false;
    }
    sprintf(arr, "%02X", c);
    hexString.append(arr, arr + 2);
  }

  return hexString;
}

int H264Unpacketizer::ParseStapAOrSingleNalu(
    const std::vector<char> &vBufReceive) {
  int ret = 0;
  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(vBufReceive.data());
  char const *const payload = vBufReceive.data() + rtpHeader.getHeaderLength();
  int length = vBufReceive.size() - rtpHeader.getHeaderLength();

  const char *nalu_start = payload + kNalHeaderSize;
  const size_t nalu_length = length - kNalHeaderSize;
  enVideoH264NaluType nal_type =
      static_cast<enVideoH264NaluType>(payload[0] & kH264TypeMask);
  std::vector<size_t> nalu_start_offsets;
  if (nal_type == kH264StapA) {
    // Skip the StapA header (StapA NAL type + length).
    if (length <= kStapAHeaderSize) {
      tylog("error: length=%d <= kStapAHeaderSize=%d", length,
            kStapAHeaderSize);

      return -1;
    }

    ret = ParseStapAStartOffsets(nalu_start, nalu_length, &nalu_start_offsets);
    if (ret) {
      tylog("parseStapAStartOffsets ret=%d", ret);

      return ret;
    }

    nal_type = static_cast<enVideoH264NaluType>(payload[kStapAHeaderSize] &
                                                kH264TypeMask);
    // no use?
  } else {
    nalu_start_offsets.push_back(0);
  }

  if (frame_buffer_.frames.empty() ||
      frame_buffer_.frames.back().timestamp != rtpHeader.getTimestamp()) {
    frame_buffer_.PushNewFrame(rtpHeader);
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
    nalu.type =
        static_cast<enVideoH264NaluType>(payload[start_offset] & kH264TypeMask);
    tylog("nalu.type=%s", enVideoH264NaluTypeToString(nalu.type).data());
    nalu.offset = start_offset;
    nalu.size = end_offset - start_offset;
    // start_offset += kNaluTypeSize;

    assert(!frame_buffer_.frames.empty());

    FrameItem &frame = frame_buffer_.frames.back();

    switch (nalu.type) {
      case kVideoNaluSps:
        ret = UpdataSps(payload + start_offset, nalu.size);
        if (ret) {
          return ret;
        }
        break;

      case kVideoNaluPps:
        ret = UpdataPps(payload + start_offset, nalu.size);
        if (ret) {
          return ret;
        }
        break;

      case kVideoNaluIdr:
        tylog("recv 264 IDR frame");

        if (unpack_params_.sps.empty()) {
          tylog("get IDR, but have no SPS before");
          return -2;
        }
        if (unpack_params_.pps.empty()) {
          tylog("get IDR, but have no PPS before");
          return -3;
        }

        if (frame.rawData.empty()) {
          // 一般之前有sps, pps 在switch结构后会加
          frame.rawData.append(kH264StartCodeCharArray)
              .append(unpack_params_.sps)
              .append(kH264StartCodeCharArray)
              .append(unpack_params_.pps);
        }

      case kVideoNaluSlice:
        break;
      // Slices below don't contain SPS or PPS ids.
      case kVideoNaluSei:
      case kVideoNaluUnitDelimiterRbsp:
      case kVideoNaluUnitEoseq:
      case kVideoNaluUnitEostm:
      case kVideoNaluUnitSpsExtn:
        break;
      case kH264StapA:
      case kH264FuA:
        tylog("error: after parse, recv %s",
              enVideoH264NaluTypeToString(nalu.type).data());
        return -4;

      default:
        tylog("warning: recv nalu type=%s",
              enVideoH264NaluTypeToString(nalu.type).data());
        break;
    }

    // should check buffer overflow
    frame.rawData.append(kH264StartCodeCharArray)
        .append(payload + start_offset, nalu.size);
  }

  return 0;
}

int H264Unpacketizer::Unpacketize(const std::vector<char> &vBufReceive,
                                  std::vector<MediaData> *o_mediaList) {
  int ret = 0;

  uint32_t lost_pkt_cnt = 0;
  const RtpHeader &rtpHeader =
      *reinterpret_cast<const RtpHeader *>(vBufReceive.data());

  if (!started_) {
    started_ = true;
    last_seq_num_ = rtpHeader.getSeqNumber();
  }

  if (AheadOf(last_seq_num_, rtpHeader.getSeqNumber())) {
    tylog("warning: last seq num=%d > current seq=%d, should be <=",
          last_seq_num_, rtpHeader.getSeqNumber());

    return -1;
  }

  if (last_seq_num_ != rtpHeader.getSeqNumber()) {
    tylog(
        "NOTE: find lost pkt(%d), last pkt(%d), have fus, buf still not find "
        "fu-end!",
        rtpHeader.getSeqNumber(), last_seq_num_ + 1);  // ?

    last_seq_num_ = rtpHeader.getSeqNumber() + 1;
    // return frames;
  }
  ++last_seq_num_;  // ?

  // status_.in_buf_cnt++; // just for monitor
  // VideoUnPackCheckLost(rtp_data, rtp_len, lost_pkt_cnt);

  unpack_params_.cur_rtp_seq_no =
      (unpack_params_.cur_rtp_seq_no + lost_pkt_cnt) & 0x00FFFF;

  //发现webrtc收到全padding的数据，此数据非264数据，丢弃
  if (static_cast<int>(vBufReceive.size()) <=
      getRtpPaddingLength(vBufReceive) + rtpHeader.getHeaderLength() + 1) {
    tylog("maybe all padding data");

    if (static_cast<int>(vBufReceive.size()) !=
        getRtpPaddingLength(vBufReceive) + rtpHeader.getHeaderLength()) {
      tylog(
          "receive an unknow rtp with padding, rtp=%s, "
          "paddinglen=%d, "
          "recv buf len=%zu.",
          rtpHeader.ToString().data(), getRtpPaddingLength(vBufReceive),
          vBufReceive.size());
    }

    // not error
    return 0;
  }

  char const *const payload = vBufReceive.data() + rtpHeader.getHeaderLength();
  enVideoH264NaluType nal_type =
      static_cast<enVideoH264NaluType>(payload[0] & kH264TypeMask);
  tylog("nalu type=%d[%s]", nal_type,
        enVideoH264NaluTypeToString(nal_type).data());
  if (nal_type == kH264FuA) {
    ret = ParseFuaNalu(vBufReceive);
    if (ret) {
      tylog("parseFuaNalu ret=%d, hexString=%s.", ret,
            ConvertToReadableHex(vBufReceive).data());

      return ret;
    }
  } else {
    ret = ParseStapAOrSingleNalu(vBufReceive);
    if (ret) {
      tylog("parseStapAOrSingleNalu ret=%d, hexString=%s.", ret,
            ConvertToReadableHex(vBufReceive).data());

      return ret;
    }
  }

  if (rtpHeader.getMarker()) {
    tylog("key: rtp mark=1");
    // use move, To check
    *o_mediaList = frame_buffer_.PopFrames();
  }

  // status_.recv_frame_cnt += frames.size();
  return 0;
}

int H264Unpacketizer::UpdataSps(const char *data, size_t len) {
  if (len >= VIDEO_MAX_SPP_PPS_LEN) {
    tylog("error: recv sps len=%zu too big", len);
    return -1;
  }
  unpack_params_.sps.assign(data, len);

  tylog("recv sps len=%zu", len);

  return 0;
}

int H264Unpacketizer::UpdataPps(const char *data, size_t len) {
  if (len >= VIDEO_MAX_SPP_PPS_LEN) {
    tylog("error: recv pps len=%zu too big", len);
    return -1;
  }
  unpack_params_.pps.assign(data, len);

  tylog("recv pps len=%zu", len);

  return 0;
}

int H264Unpacketizer::DumpRawStream(const std::string &rawStream,
                                    uint32_t ssrc) {
  if (rtp_2_h264_file_ == nullptr) {
    // /for debug, not use ssrc
    // now allow only one pc
    (void)ssrc;
    const std::string &name = tylib::format_string("uplink_ssrc_%u.h264", 0);
    rtp_2_h264_file_ = fopen(name.data(), "wb+");
    if (nullptr == rtp_2_h264_file_) {
      tylog("open file %s fail, errno=%d[%s]", name.data(), errno,
            strerror(errno));
      return -1;
    }
  }

  // write 1 item, whose size is rawStream.size()
  size_t n = fwrite(rawStream.data(), rawStream.size(), 1, rtp_2_h264_file_);
  if (n < 1) {
    tylog("fwrite fail, return value=%zu.", n);
    return -2;
  }

  return 0;
}
