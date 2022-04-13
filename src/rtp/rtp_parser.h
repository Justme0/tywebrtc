// from licode

#ifndef RTP_RTP_PARSER_H_
#define RTP_RTP_PARSER_H_

#include <netinet/in.h>

#include "log/log.h"
#include "tylib/string/format_string.h"

// The Internet Protocol defines big-endian as the standard network byte order

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
        ssrc(0),
        extensionpayload(0),
        extensionlength(0) {}

  uint8_t hasPadding() const { return padding; }

  void setPadding(uint8_t has_padding) { padding = has_padding; }

  uint8_t getVersion() const { return version; }
  void setVersion(uint8_t aVersion) { version = aVersion; }
  uint8_t getMarker() const { return marker; }
  void setMarker(uint8_t aMarker) { marker = aMarker; }
  uint8_t getExtension() const { return hasextension; }
  void setExtension(uint8_t ext) { hasextension = ext; }
  uint8_t getCc() const { return cc; }
  void setCc(uint8_t theCc) { cc = theCc; }
  uint8_t getPayloadType() const { return payloadtype; }
  void setPayloadType(uint8_t aType) { payloadtype = aType; }
  uint16_t getSeqNumber() const { return ntohs(seqnum); }
  void setSeqNumber(uint16_t aSeqNumber) { seqnum = htons(aSeqNumber); }
  uint32_t getTimestamp() const { return ntohl(timestamp); }
  void setTimestamp(uint32_t aTimestamp) { timestamp = htonl(aTimestamp); }
  uint32_t getSSRC() const { return ntohl(ssrc); }
  void setSSRC(uint32_t aSSRC) { ssrc = htonl(aSSRC); }
  uint16_t getExtId() const { return ntohs(extensionpayload); }
  void setExtId(uint16_t extensionId) { extensionpayload = htons(extensionId); }
  uint16_t getExtLength() const { return ntohs(extensionlength); }
  void setExtLength(uint16_t extensionLength) {
    extensionlength = htons(extensionLength);
  }
  int getHeaderLength() const {
    static const int MIN_SIZE = 12;
    return MIN_SIZE + cc * 4 + hasextension * (4 + ntohs(extensionlength) * 4);
  }

  std::string ToString() const {
    return tylib::format_string(
        "{payload_type=%u, marker=%u, sequence_number=%u, padding_size=%u, "
        "timestamp=%u, ssrc=%u, payload_offset=%u, payload_size=%u, "
        "total_size=%u}",
        getPayloadType(), getMarker(), getSeqNumber(), 22222, getTimestamp(),
        getSSRC(), 22222, 22222, 2222);  // taylor to fix
  }

 private:
  // the following is all netorder for RAW RTP format
  uint32_t cc : 4;
  uint32_t hasextension : 1;
  uint32_t padding : 1;
  uint32_t version : 2;
  uint32_t payloadtype : 7;
  uint32_t marker : 1;
  uint32_t seqnum : 16;
  uint32_t timestamp;
  uint32_t ssrc;
  uint32_t extensionpayload : 16;
  uint32_t extensionlength : 16;

  // uint32_t extensions;
};

// string enum, for print convenience
const std::string& kMediaTypeRtcp = "rtcp";
const std::string& kMediaTypeVideo = "video";
const std::string& kMediaTypeAudio = "audio";

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

  static uint16_t getSeqNum(const void* packet, size_t bytes) {
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

  static uint32_t getTimeStamp(const void* packet, size_t bytes) {
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