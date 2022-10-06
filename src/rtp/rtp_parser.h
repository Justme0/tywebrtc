// from licode

#ifndef RTP_RTP_PARSER_H_
#define RTP_RTP_PARSER_H_

#include <netinet/in.h>

#include <limits>

#include "log/log.h"
#include "tylib/string/format_string.h"

#define VIDEO_RTP_EXTERN_NAME_LEN (2)   // 扩展位名字
#define VIDEO_RTP_EXTERN_VALUE_LEN (4)  // 扩展数据长度单位为4个字节
#define VIDEO_RTP_EXTERN_LEN_VALUE_LEN (2)  // 扩展数据长度占据2字节

extern const std::string kMediaTypeRtcp;
extern const std::string kMediaTypeVideo;
extern const std::string kMediaTypeAudio;

// to move to tylib
inline void WriteBigEndian(uint8_t* data, uint32_t val, int max_byte) {
  int i;
  for (i = 0; i < max_byte; ++i) {
    data[i] = val >> ((max_byte - 1 - i) * 8);
  }
}

inline uint32_t ReadBigEndian(const uint8_t* data, int max_byte) {
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

  bool Parse(const uint8_t* data, size_t size) {
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
  bool Write(uint8_t* data, size_t size) {
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

enum MediaType { kMediaVideo, kMediaAudio, kMediaData, kMediaMax };

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

  // return RTP head length.
  // include 3 parts: Fix Header; CSRC; extension
  int getHeaderLength() const {
    const int csrcLen = cc * 4;
    const int extensionLen =
        hasextension ? this->getHeaderExt()->getExtWholeLength() : 0;
    return kRtpHeaderLenByte + csrcLen + extensionLen;
  }

  // taylor FIX must parse
  // return ptr for multi-type
  std::vector<std::shared_ptr<Extension>> getParsedExtensions() const {
    std::vector<std::shared_ptr<Extension>> ret;
    return ret;
  }

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
  uint32_t cc : 4;
  uint32_t hasextension : 1;
  uint32_t padding : 1;
  uint32_t version : 2;
  uint32_t payloadtype : 7;
  uint32_t marker : 1;

  // all are net order
  uint32_t seqnum : 16;
  uint32_t timestamp;
  uint32_t ssrc;
  // ... csrc
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

  // taylor RTP should check according to SSRC
  static int GetMediaType(const std::vector<char>& vBufReceive,
                          std::string* o_mediaType) {
    int ret = 0;

    uint8_t payloadTypeChar = 0;
    ret = RtpRtcpStrategy::GetPayloadType(vBufReceive, &payloadTypeChar);
    if (ret) {
      tylog("get payload fail, ret=%d", ret);
      return ret;
    }

    bool bRtcp = RtpRtcpStrategy::isRTCP(payloadTypeChar);
    if (bRtcp) {
      tylog("recv pt=%u, is rtcp", payloadTypeChar);
      *o_mediaType = kMediaTypeRtcp;
      return 0;
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
        tylog("recv pt=%d, is video", kRtpPayloadType);
        *o_mediaType = kMediaTypeVideo;
        return 0;

      case 103:
      case 104:
      case 111:
        tylog("recv pt=%d, is audio", kRtpPayloadType);
        *o_mediaType = kMediaTypeAudio;
        return 0;
    }

    tylog("recv pt(char)=%#x, unknown char", payloadTypeChar);

    return -1;
  }

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
  // @brief check if RTCP
  // 72 to 76 is reserved for RTP
  // 77 to 79 is not reserver but they are not assigned we will block them
  // for RTCP 200 SR  == marker bit + 72
  // for RTCP 204 APP == marker bit + 76
  //
  //       RTCP
  //
  // FIR      full INTRA-frame request             192     [RFC2032] supported
  // NACK     negative acknowledgement             193     [RFC2032]
  // IJ       Extended inter-arrival jitter report 195
  // [RFC-ietf-avt-rtp-toffset-07.txt]
  // http://tools.ietf.org/html/draft-ietf-avt-rtp-toffset-07 SR       sender
  // report                        200     [RFC3551] supported RR receiver
  // report                      201     [RFC3551] supported SDES     source
  // description                   202     [RFC3551] supported BYE goodbye 203
  // [RFC3551] supported APP      application-defined                  204
  // [RFC3551] ignored RTPFB    Transport layer FB message           205
  // [RFC4585] supported PSFB     Payload-specific FB message          206
  // [RFC4585] supported XR       extended report                      207
  // [RFC3611] supported
  // 205       RFC 5104
  // FMT 1      NACK       supported
  // FMT 2      reserved
  // FMT 3      TMMBR      supported
  // FMT 4      TMMBN      supported
  // 206      RFC 5104
  // FMT 1:     Picture Loss Indication (PLI)                      supported
  // FMT 2:     Slice Lost Indication (SLI)
  // FMT 3:     Reference Picture Selection Indication (RPSI)
  // FMT 4:     Full Intra Request (FIR) Command                   supported
  // FMT 5:     Temporal-Spatial Trade-off Request (TSTR)
  // FMT 6:     Temporal-Spatial Trade-off Notification (TSTN)
  // FMT 7:     Video Back Channel Message (VBCM)
  // FMT 15:    Application layer FB message
  static bool isRTCP(uint8_t payloadType) {
    switch (payloadType) {
      case 192:
      case 193:
      case 195:
      case 200:
      case 201:
      case 202:
      case 203:
      case 204:
      case 205:
      case 206:
      case 207:
        return true;
      default:
        return false;
    }
  }

 protected:
  unsigned int uiLocalAudioSsrc;
  unsigned int uiLocalVideoSsrc;
  unsigned int uiLocalRtxSsrc;

  unsigned int uiRemoteAudioSsrc;
  unsigned int uiRemoteVideoSsrc;
  unsigned int uiRemoteRtxSsrc;
};

#endif  // RTP_RTP_PARSER_H_