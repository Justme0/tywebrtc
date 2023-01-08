// from licode

#ifndef RTP_RTP_PARSER_H_
#define RTP_RTP_PARSER_H_

#include <netinet/in.h>

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "tylib/string/any_to_string.h"
#include "tylib/string/format_string.h"
#include "tylib/time/time_util.h"

#include "global_tmp/global_tmp.h"
#include "log/log.h"
#include "rtp/rtcp/rtcp_parser.h"

// define cycle as int64_t (use 47 bit at most, most significant bit for sign
// if need), first value is 0. e.g. 8Mbps, each video packet is 1000 Byte, so
// 8*2^20/8/1000=2^10 packets per second, 2^(47+16)/2^10/3600/24/365 =
// 285616414 year, it's enough.
// common case one cycle: 550kbps, 955B/s, 65535/(550000/8/955)/60=15min
using PowerSeqT = int64_t;

inline std::pair<int64_t, uint16_t> SplitPowerSeq(PowerSeqT powerSeq) {
  int64_t cycle = powerSeq >> 16;
  uint16_t seq = powerSeq & 0xFFFF;

  return std::make_pair(cycle, seq);
}

inline std::string PowerSeqToString(PowerSeqT powerSeq) {
  return tylib::AnyToString(SplitPowerSeq(powerSeq));
}

#define VIDEO_RTP_EXTERN_NAME_LEN (2)   // 扩展位名字
#define VIDEO_RTP_EXTERN_VALUE_LEN (4)  // 扩展数据长度单位为4个字节
#define VIDEO_RTP_EXTERN_LEN_VALUE_LEN (2)  // 扩展数据长度占据2字节

extern const std::string kMediaTypeRtcp;
extern const std::string kMediaTypeVideo;
extern const std::string kMediaTypeAudio;

// @return "b - a"
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

// @return if "a > b"
template <typename T, T M>
inline typename std::enable_if<(M == 0), bool>::type AheadOrAt(T a, T b) {
  static_assert(std::is_unsigned<T>::value,
                "Type must be an unsigned integer.");
  const T maxDist = std::numeric_limits<T>::max() / 2 + T(1);
  if (a - b == maxDist) return b < a;
  return ForwardDiff(b, a) < maxDist;  // 0 < a-b < maxDist
}

template <typename T>
inline bool AheadOrAt(T a, T b) {
  return AheadOrAt<T, 0>(a, b);
}

// @return if "a > b"
template <typename T, T M = 0>
inline bool AheadOf(T a, T b) {
  static_assert(std::is_unsigned<T>::value,
                "Type must be an unsigned integer.");
  return a != b && AheadOrAt<T, M>(a, b);
}

// to move to tylib
inline void WriteBigEndian(char* data, uint32_t val, int max_byte) {
  int i;
  for (i = 0; i < max_byte; ++i) {
    data[i] = val >> ((max_byte - 1 - i) * 8);
  }
}

inline uint32_t ReadBigEndian(const char* data, int max_byte) {
  uint32_t val = 0;
  int i;
  for (i = 0; i < max_byte; ++i) {
    val |= (uint32_t)(data[i]) << ((max_byte - 1 - i) * 8);
  }
  return val;
}

// Clockwise rotation
enum VideoRotation {
  kVideoRotation0 = 0,
  kVideoRotation90 = 1,
  kVideoRotation180 = 2,
  kVideoRotation270 = 3,
  kVideoRotationButt
};

// The Internet Protocol defines big-endian as the standard network byte order

enum enRtpExtType {
  kRtpExtNone = 0,
  // urn:ietf:params:rtp-hdrext:ssrc-audio-level
  kRtpExtAudioLevel = 1,
  // urn:ietf:params:rtp-hdrext:toffset
  kRtpExtTransmissionTimeOffset = 2,
  // http://www.webrtc.org/experiments/rtp-hdrext/abs-send-time
  kRtpExtAbsoluteSendTime = 3,
  // urn:3gpp:video-orientation
  kRtpExtVideoRotation = 4,
  // http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01
  kRtpExtTransportSeq = 5,
  // http://www.webrtc.org/experiments/rtp-hdrext/playout-delay
  kRtpExtPlayoutDelay = 6,
  // http://www.webrtc.org/experiments/rtp-hdrext/video-content-type
  kRtpExtContentType = 7,
  // urn:ietf:params:rtp-hdrext:sdes:rtp-stream-id
  kRtpExtRtpStreamId = 8,
  // http://www.webrtc.org/experiments/rtp-hdrext/video-timing
  kRtpExtVideoTiming = 9,
  // http://tools.ietf.org/html/draft-ietf-avtext-framemarking-07
  kRtpExtFramemarking = 10,
  // http://www.webrtc.org/experiments/rtp-hdrext/color-space
  kRtpExtColorPpace = 11,
  // urn:ietf:params:rtp-hdrext:sdes:repaired-rtp-stream-id
  kRtpExtRepairedRtpStreamId,
  // urn:ietf:params:rtp-hdrext:sdes:rtp-stream-id
  kRtpExtMediaStreamId,
  // uri:webrtc:rtc:rtp-hdrext:video:CompositionTime
  kRtpExtCompositionTime = 58,
  kRtpExtNumberOfExtensions,  // Must be the last entity in the enum. max value
                              // 16
  // kMaxRtpExtNumber = 16
  kMaxRtpExtNumber =
      256  // 256 appbits https://datatracker.ietf.org/doc/html/rfc5285
};

// base class
struct Extension {
  enRtpExtType extension_type;
};

class AudioLevelExt : public Extension {
 public:
  bool voice_activity;
  uint8_t audio_level;
};

class TransmissionTimeOffsetExt : public Extension {
 public:
  int32_t transmission_time_offset;
};

class AbsoluteSendTimeExt : public Extension {
 public:
  uint32_t absolute_send_time;
};

class VideoRotationExt : public Extension {
 public:
  VideoRotation video_rotation;
};

class TransportSequenceNumberExt : public Extension {
 public:
  uint16_t transport_sequence_number;
};

class VideoContentTypeExt : public Extension {
 public:
  uint8_t video_content_type;
};

class FrameMarkingExt : public Extension {
 public:
  struct FrameMarks {
    bool start_frame = false;
    bool end_frame = false;
    bool independent = false;
    bool discardable = false;
    bool base_layer_sync = false;
    uint8_t temporal_layer_id = 0;
    uint8_t layer_id = 0;
    uint8_t tl0_pic_idx = 0;
  };
  FrameMarks frame_marks_;
};

class RtpStreamIdExt : public Extension {
 public:
  std::string rid;
};

class RepairedRtpStreamIdExt : public Extension {
 public:
  std::string repaired_rid;
};

class MediaStreamIdExt : public Extension {
 public:
  std::string mid;
};

class CompositionTimeIdExt : public Extension {
 public:
  int cts = 0;
};

// Video timing timestamps in ms counted from capture_time_ms of a frame.
// This structure represents data sent in video-timing RTP header extension.
class VideoTimingIdExt : public Extension {
 public:
  enum TimingFrameFlags : uint8_t {
    kNotTriggered = 0,  // Timing info valid, but not to be transmitted.
                        // Used on send-side only.
    kTriggeredByTimer = 1 << 0,  // Frame marked for tracing by periodic timer.
    kTriggeredBySize = 1 << 1,   // Frame marked for tracing due to size.
    kInvalid = std::numeric_limits<uint8_t>::max()  // Invalid, ignore!
  };

  static constexpr uint8_t kValueSizeBytes = 13;
  static constexpr uint8_t kFlagsOffset = 0;
  static constexpr uint8_t kEncodeStartDeltaOffset = 1;
  static constexpr uint8_t kEncodeFinishDeltaOffset = 3;
  static constexpr uint8_t kPacketizationFinishDeltaOffset = 5;
  static constexpr uint8_t kPacerExitDeltaOffset = 7;
  static constexpr uint8_t kNetworkTimestampDeltaOffset = 9;
  static constexpr uint8_t kNetwork2TimestampDeltaOffset = 11;

  uint16_t encode_start_delta_ms;
  uint16_t encode_finish_delta_ms;
  uint16_t packetization_finish_delta_ms;
  uint16_t pacer_exit_delta_ms;
  uint16_t network_timestamp_delta_ms;
  uint16_t network2_timestamp_delta_ms;
  uint8_t flags = TimingFrameFlags::kInvalid;
};

// Video Timing.
// 6 timestamps in milliseconds counted from capture time stored in rtp header:
// encode start/finish, packetization complete, pacer exit and reserved for
// modification by the network modification. |flags| is a bitmask and has the
// following allowed values:
// 0 = Valid data, but no flags available (backwards compatibility)
// 1 = Frame marked as timing frame due to cyclic timer.
// 2 = Frame marked as timing frame due to size being outside limit.
// 255 = Invalid. The whole timing frame extension should be ignored.
//
//    0                   1                   2                   3
//    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |  ID   | len=12|     flags     |     encode start ms delta     |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |    encode finish ms delta     |  packetizer finish ms delta   |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |     pacer exit ms delta       |  network timestamp ms delta   |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//   |  network2 timestamp ms delta  |
//   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class VideoSendTiming : public Extension {
  // Returns |time_ms - base_ms| capped at max 16-bit value.
  // Used to fill this data structure as per
  // https://webrtc.org/experiments/rtp-hdrext/video-timing/ extension stores
  // 16-bit deltas of timestamps from packet capture time.
 public:
  const uint8_t kValueSizeBytes = 13;
  // Offsets of the fields in the RTP header extension, counting from the first
  // byte after the one-byte header.
  const uint8_t kFlagsOffset = 0;
  const uint8_t kEncodeStartDeltaOffset = 1;
  const uint8_t kEncodeFinishDeltaOffset = 3;
  const uint8_t kPacketizationFinishDeltaOffset = 5;
  const uint8_t kPacerExitDeltaOffset = 7;
  const uint8_t kNetworkTimestampDeltaOffset = 9;
  const uint8_t kNetwork2TimestampDeltaOffset = 11;

  bool Parse(const char* data, size_t size) {
    int64_t off = 0;
    if (static_cast<uint8_t>(size) == kValueSizeBytes - 1) {
      flags = 0;
      off = 1;  // Old wire format without the flags field.
    } else if (size == kValueSizeBytes) {
      flags = ReadBigEndian(data, 1);
    } else {
      return false;
    }

    encode_start_delta_ms =
        ReadBigEndian(data + kEncodeStartDeltaOffset - off, 2);
    encode_finish_delta_ms =
        ReadBigEndian(data + kEncodeFinishDeltaOffset - off, 2);
    packetization_finish_delta_ms =
        ReadBigEndian(data + kPacketizationFinishDeltaOffset - off, 2);
    pacer_exit_delta_ms = ReadBigEndian(data + kPacerExitDeltaOffset - off, 2);
    network_timestamp_delta_ms =
        ReadBigEndian(data + kNetworkTimestampDeltaOffset - off, 2);
    network2_timestamp_delta_ms =
        ReadBigEndian(data + kNetwork2TimestampDeltaOffset - off, 2);
    // WEBRTC_LOG_ERROR("123456","videoTiming parse: flags
    // %d,encode_start_delta_ms %d,encode_finish_delta_ms
    // %d,packetization_finish_delta_ms %d,pacer_exit_delta_ms
    // %d,network_timestamp_delta_ms %d,network2_timestamp_delta_ms %d",
    // flags,encode_start_delta_ms,encode_finish_delta_ms,packetization_finish_delta_ms,pacer_exit_delta_ms,network_timestamp_delta_ms,network2_timestamp_delta_ms);
    return true;
  }
  bool Write(char* data, size_t size) {
    if (size != kValueSizeBytes) {
      return false;
    }
    WriteBigEndian(data + kFlagsOffset, flags, 1);
    WriteBigEndian(data + kEncodeStartDeltaOffset, encode_start_delta_ms, 2);
    WriteBigEndian(data + kEncodeFinishDeltaOffset, encode_finish_delta_ms, 2);
    WriteBigEndian(data + kPacketizationFinishDeltaOffset,
                   packetization_finish_delta_ms, 2);
    WriteBigEndian(data + kPacerExitDeltaOffset, pacer_exit_delta_ms, 2);
    WriteBigEndian(data + kNetworkTimestampDeltaOffset,
                   network_timestamp_delta_ms, 2);
    WriteBigEndian(data + kNetwork2TimestampDeltaOffset,
                   network2_timestamp_delta_ms, 2);
    // WEBRTC_LOG_ERROR("123456","videoTiming write: flags
    // %d,encode_start_delta_ms %d,encode_finish_delta_ms
    // %d,packetization_finish_delta_ms %d,pacer_exit_delta_ms
    // %d,network_timestamp_delta_ms %d,network2_timestamp_delta_ms %d",
    // flags,encode_start_delta_ms,encode_finish_delta_ms,packetization_finish_delta_ms,pacer_exit_delta_ms,network_timestamp_delta_ms,network2_timestamp_delta_ms);
    return true;
  }

 private:
  uint16_t encode_start_delta_ms;
  uint16_t encode_finish_delta_ms;
  uint16_t packetization_finish_delta_ms;
  uint16_t pacer_exit_delta_ms;
  uint16_t network_timestamp_delta_ms;
  uint16_t network2_timestamp_delta_ms;
  uint8_t flags;
};

// enum MediaType { kMediaVideo, kMediaAudio, kMediaData, kMediaMax };

static inline uint16_t rtp_read_uint16(const uint8_t* ptr) {
  return (((uint16_t)ptr[0]) << 8) | ptr[1];
}

static inline uint32_t rtp_read_uint32(const uint8_t* ptr) {
  return (((uint32_t)ptr[0]) << 24) | (((uint32_t)ptr[1]) << 16) |
         (((uint32_t)ptr[2]) << 8) | ptr[3];
}

static inline void rtp_write_uint32(uint8_t* ptr, uint32_t val) {
  ptr[0] = (uint8_t)((val >> 24) & 0xFF);
  ptr[1] = (uint8_t)((val >> 16) & 0xFF);
  ptr[2] = (uint8_t)((val >> 8) & 0xFF);
  ptr[3] = (uint8_t)(val & 0xFF);
}

class PowerfulSeq {};

//  0                   1                   2                   3
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |V=2|P|X|  CC   |M|     PT      |       sequence number         |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                           timestamp                           |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |           synchronization source (SSRC) identifier            |
// +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
// |            contributing source (CSRC) identifiers             |
// |                             ....                              |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

//  0                   1                   2                   3
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |      defined by profile       |           length              |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                        header extension                       |
// |                             ....                              |
class RtpFixedHeaderExt {
 public:
  RtpFixedHeaderExt() : extensionpayload(0), extensionlength(0) {}

  uint16_t getExtId() const { return ntohs(extensionpayload); }
  void setExtId(uint16_t extensionId) { extensionpayload = htons(extensionId); }

  // indicates the length of the extension in 32-bit units, excluding the 32
  // bits of the extension header
  // https://en.wikipedia.org/wiki/Real-time_Transport_Protocol
  uint16_t getExtLength() const { return ntohs(extensionlength); }
  void setExtLength(uint16_t extensionlength) {
    extensionlength = htons(extensionlength);
  }

  int getExtWholeLength() const {
    const int kSelfHeadExtLen = 4;

    // 4 * this->getExtLength() should check:
    // profile == 0xBEDE || (profile & 0xFFF0) == 0x1000
    // ref: https://www.rfc-editor.org/rfc/rfc5285#section-4
    return kSelfHeadExtLen + 4 * this->getExtLength();
  }

 private:
  // all are net order
  uint16_t extensionpayload;
  uint16_t extensionlength;

  // uint32_t extensions;
};

const int kRtpHeaderLenByte = 12;

class RtpHeader {
 public:
  RtpHeader()
      : cc(0),
        hasextension(0),
        padding(0),
        version(2),
        payloadtype(0),
        marker(0),
        seqnum(0),
        timestamp(0),
        ssrc(0) {}

  uint8_t getCc() const { return cc; }
  void setCc(uint8_t theCc) { cc = theCc; }

  uint8_t getExtension() const { return hasextension; }
  void setExtension(uint8_t ext) { hasextension = ext; }

  uint8_t hasPadding() const { return padding; }
  void setPadding(uint8_t has_padding) { padding = has_padding; }

  uint8_t getVersion() const { return version; }
  void setVersion(uint8_t aVersion) { version = aVersion; }

  uint8_t getPayloadType() const { return payloadtype; }
  void setPayloadType(uint8_t aType) { payloadtype = aType; }

  uint8_t getMarker() const { return marker; }
  void setMarker(uint8_t aMarker) { marker = aMarker; }

  uint16_t getSeqNumber() const { return ntohs(seqnum); }
  void setSeqNumber(uint16_t aSeqNumber) { seqnum = htons(aSeqNumber); }

  uint32_t getTimestamp() const { return ntohl(timestamp); }
  void setTimestamp(uint32_t aTimestamp) { timestamp = htonl(aTimestamp); }

  uint32_t getSSRC() const { return ntohl(ssrc); }
  void setSSRC(uint32_t aSSRC) { ssrc = htonl(aSSRC); }

  /*
    // get extension msg need check csrc
    uint16_t getExtId() const {
      RtpFixedHeaderExt* rtpext = getHeaderExt();
      if (rtpext) {
        return ntohs(rtpext->extensionpayload);
      }
      return ntohs(extensionpayload);
    }

    uint16_t getExtLength() const {
      RtpFixedHeaderExt* rtpext = getHeaderExt();
      if (rtpext) {
        return ntohs(rtpext->length);
      }
      return ntohs(extensionlength);
    }
    */

  // get RTP head length.
  // include 3 parts: Fix Header; CSRC; extension
  int getHeaderLength() const {
    const int csrcLen = cc * 4;
    const int extensionLen =
        hasextension ? this->getHeaderExt()->getExtWholeLength() : 0;
    return kRtpHeaderLenByte + csrcLen + extensionLen;
  }

  // taylor FIX must parse
  // return ptr for multi-type, key
  std::vector<std::shared_ptr<Extension>> parseExtensions() const { return {}; }

  /*
    inline int GetRtpExtOffset(char* pRtpData, int RtpLen) {
      int CSRCOffset = getCc() << 2;

      // 如果x=1则包含 4个字节类型 2个字节扩展长度(4个字节作为单位)
      int ExternOffset = 0;
      int ExternLen = 0;

      if (this->getMarker()) {
          const char* pBuff = this + kRtpHeaderLenByte +  CSRCOffset;
          const RtpFixedHeaderExt* pRtpExtHeader = reinterpret_cast<const
  RtpFixedHeaderExt*>(pBuff);

          ExternLen = pRtpExtHeader->getExtLength() << 2;
          ExternOffset = VIDEO_RTP_EXTERN_NAME_LEN +
  VIDEO_RTP_EXTERN_LEN_VALUE_LEN + ExternLen;
      }

      int RealOffset = ExternOffset + CSRCOffset;

      return RealOffset;
  }
  */

  // compatible with RTCP.
  // taylor RTP should check according to SSRC,
  // maybe rtcp (payload is 8 bit).
  // bug:
  // if the char is RTP payload 95, mark = 1, then char is 223,
  // checked to rtcp type: RTCP_MAX_PT(223)
  std::string GetMediaType() const {
    uint8_t payloadTypeChar = reinterpret_cast<const char*>(this)[1];
    bool bRtcp = isRtcp(static_cast<RtcpPacketType>(payloadTypeChar));
    if (bRtcp) {
      return kMediaTypeRtcp;
    }

    // remove first MARK bit
    const uint8_t kRtpPayloadType = payloadTypeChar & 0x7F;
    switch (kRtpPayloadType) {
      case 96:
      case 97:
      case 98:
      case 99:
      case 100:
      case 101:
      case 107:
      case 116:
      case 117:
      case 125:
      case 127:
        return kMediaTypeVideo;

      case 103:
      case 104:
      case 111:
        return kMediaTypeAudio;
    }

    return "";
  }

  std::string ToString() const {
    return tylib::format_string(
        "{CSRC count=%d, has ext=%d, has padding=%d, version=%d, "
        "payload_type=%u, marker=%u, sequence_number=%u, timestamp=%u, "
        "ssrc=%u(0x%X), head size=%d}",
        getCc(), getExtension(), hasPadding(), getVersion(), getPayloadType(),
        getMarker(), getSeqNumber(), getTimestamp(), getSSRC(), getSSRC(),
        getHeaderLength());
  }

 private:
  // if header ext exists, return value is the address of header ext struct.
  // Otherwise undefined behavior
  const RtpFixedHeaderExt* getHeaderExt() const {
    return (RtpFixedHeaderExt*)((uint8_t*)this + kRtpHeaderLenByte + cc * 4);
  }

 private:
  // the following is all netorder for RAW RTP format
  uint8_t cc : 4;
  uint8_t hasextension : 1;
  uint8_t padding : 1;
  uint8_t version : 2;
  uint8_t payloadtype : 7;
  uint8_t marker : 1;

  // all are net order
  uint16_t seqnum;
  uint32_t timestamp;
  uint32_t ssrc;
  // ... csrc
};

// RTP packet with biz info.
// include raw packet and biz info(seq cycle, CSRC ...)
struct RtpBizPacket {
  // RtpBizPacket() = default;

  // should avoid copy big blob data
  RtpBizPacket(const RtpBizPacket&) = delete;
  RtpBizPacket& operator=(const RtpBizPacket& other) = delete;

  RtpBizPacket(std::vector<char>&& rtpRawPacket, int64_t cycle)
      : rtpRawPacket(std::move(rtpRawPacket)), cycle(cycle) {}

  // https://learn.microsoft.com/en-us/cpp/cpp/move-constructors-and-move-assignment-operators-cpp?view=msvc-170
  RtpBizPacket(RtpBizPacket&& other) { *this = std::move(other); }

  RtpBizPacket& operator=(RtpBizPacket&& other) {
    if (this != &other) {
      rtpRawPacket = std::move(other.rtpRawPacket);
      cycle = other.cycle;
      enterTimeMs = other.enterTimeMs;
    }

    return *this;
  }

  bool operator<(const RtpBizPacket& other) const {
    return this->GetPowerSeq() < other.GetPowerSeq();
  }

  PowerSeqT GetPowerSeq() const {
    return (cycle << 16) |
           reinterpret_cast<const RtpHeader*>(rtpRawPacket.data())
               ->getSeqNumber();
  }

  std::string ToString() const {
    return tylib::format_string(
        "{rtp=%s, cycle=%ld, enterTs=%s, waitMs=%ld}",
        reinterpret_cast<const RtpHeader*>(rtpRawPacket.data())
            ->ToString()
            .data(),
        cycle, tylib::MilliSecondToLocalTimeString(enterTimeMs).data(),
        WaitTimeMs());
  }

  // from enter jitter to now duration
  int64_t WaitTimeMs() const { return g_now_ms - enterTimeMs; }

 public:  // should be private
  // when add new member, must modify constructor, ToString(), etc.
  std::vector<char> rtpRawPacket;
  int64_t cycle = 0;
  int64_t enterTimeMs = 0;
};

// padding (P): 1 bit
//       If the padding bit is set, the packet contains one or more
//       additional padding octets at the end which are not part of the
//       payload.  The last octet of the padding contains a count of how
//       many padding octets should be ignored, including itself.  Padding
//       may be needed by some encryption algorithms with fixed block sizes
//       or for carrying several RTP packets in a lower-layer protocol data
//       unit.
// OPT: define RTPPacket, make getRtpPaddingLength member function
inline int getRtpPaddingLength(const std::vector<char>& vBufReceive) {
  const RtpHeader& rtpHeader =
      *reinterpret_cast<const RtpHeader*>(vBufReceive.data());

  // pitfall: convert char to int maybe < 0
  return rtpHeader.hasPadding() ? static_cast<uint8_t>(vBufReceive.back()) : 0;
}

//  RFC 5285
//  0                   1                   2                   3
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |       0xBE    |    0xDE       |           length=3            |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |  ID   | L=0   |     data      |  ID   |  L=1  |   data...
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// ...data   |    0 (pad)    |    0 (pad)    |  ID   | L=3   |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                          data                                 |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

class RtpRtcpStrategy {
 public:
  struct RTCPSDESHeader {
    uint8_t sdesid;
    uint8_t length;
  };

  struct RTCPCommonHeader {
    uint8_t count : 5;
    uint8_t padding : 1;
    uint8_t version : 2;
    uint8_t packettype;
    uint16_t length;
  };

  /*
    static uint16_t getSeqNumShit(const void* packet, size_t bytes) {
      const uint8_t* p = (const uint8_t*)packet;
      const size_t len = bytes;

      if (len < 4) {
        return 0;
      }

      const uint8_t V = p[0] >> 6;
      if (V != 2) {
        return -1;
      }

      uint16_t seqnum = rtp_read_uint16(p + 2);
      return seqnum;
    }
    */

  /*
    static uint32_t getTimeStampShit(const void* packet, size_t bytes) {
      const uint8_t* p = (const uint8_t*)packet;
      const size_t len = bytes;

      if (len < 8) {
        return 0;
      }

      const uint8_t V = p[0] >> 6;
      if (V != 2) {
        return -1;
      }

      uint32_t timestamp = rtp_read_uint32(p + 4);
      return timestamp;
    }
    */

  // @brief get payload type
  // @param vBufReceive [in]
  // @param o_payloadTypeChar [out] if RTCP, it's real payload type; if RTP,
  // it's a char (payload type + mark bit)
  // @return 0 succ, otherwise not 0
  /*
  static int GetPayloadType(const std::vector<char>& vBufReceive,
                            uint8_t* o_payloadTypeChar) {
    if (vBufReceive.size() < 4) {
      tylog("recv buf size=%zu < 4, error?", vBufReceive.size());
      return -1;
    }

    // bit right shift, should be unsigned, so should be vector<uint8_t>
    const uint8_t version = uint8_t(vBufReceive[0]) >> 6;
    if (version != 2) {
      tylog("recv version(2 bit)=%#x, should be 2", version);

      return -2;
    }

    *o_payloadTypeChar = vBufReceive[1];

    return 0;
  }
  */

 public:
  // taylor remove the following SSRC code?
  void setLocalSSRC(unsigned int localAudioSsrc, unsigned int localVideoSsrc,
                    unsigned int localRtxSsrc) {
    uiLocalAudioSsrc = localAudioSsrc;
    uiLocalVideoSsrc = localVideoSsrc;
    uiLocalRtxSsrc = localRtxSsrc;
  }

  void setRemoteSSRC(unsigned int remoteAudioSsrc, unsigned int remoteVideoSsrc,
                     unsigned int remoteRtxSsrc) {
    uiRemoteAudioSsrc = remoteAudioSsrc;
    uiRemoteVideoSsrc = remoteVideoSsrc;
    uiRemoteRtxSsrc = remoteRtxSsrc;
  }

 private:
 protected:
  unsigned int uiLocalAudioSsrc;
  unsigned int uiLocalVideoSsrc;
  unsigned int uiLocalRtxSsrc;

  unsigned int uiRemoteAudioSsrc;
  unsigned int uiRemoteVideoSsrc;
  unsigned int uiRemoteRtxSsrc;
};

#endif  // RTP_RTP_PARSER_H_