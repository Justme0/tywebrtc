// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// RFC
// RTP & RTCP https://datatracker.ietf.org/doc/html/rfc3550#section-6
// XR https://datatracker.ietf.org/doc/html/rfc3611
//
// Ref
// licode
// https://github.com/lynckia/licode/blob/master/erizo/src/erizo/rtp/RtpHeaders.h
// wireshark
// https://github.com/wireshark/wireshark/blob/master/epan/dissectors/packet-rtcp.c

#ifndef SRC_RTP_RTCP_RTCP_PARSER_H_
#define SRC_RTP_RTCP_RTCP_PARSER_H_

#include <netinet/in.h>

#include <cstdint>

#include "tylib/string/format_string.h"

namespace tywebrtc {

// OPT: make dynamic, now reference licode
const int kSsrcFeedbackMaxNumber = 50;

enum class EnXRBlockType {
  kXRBlockRRTR = 4,
  kXRBlockDLRR = 5,
};

// payload type is 205
enum class RtcpRtpFeedbackFormat {
  kFeedbackNack = 1,
  kFeedbackTCC = 15,
};

// payload type is 206
enum class RtcpPayloadSpecificFormat {
  kRtcpPLI = 1,
  kRtcpSLI = 2,
  kRtcpRPSI = 3,
  kRtcpFIR = 4,
  kRtcpREMB = 15,
};

inline std::string RtcpPayloadSpecificFormatToString(
    RtcpPayloadSpecificFormat f) {
  switch (f) {
    case RtcpPayloadSpecificFormat::kRtcpPLI:
      return "RtcpSpecific_PLI";
    case RtcpPayloadSpecificFormat::kRtcpSLI:
      return "RtcpSpecific_SLI";
    case RtcpPayloadSpecificFormat::kRtcpRPSI:
      return "RtcpSpecific_RPSI";
    case RtcpPayloadSpecificFormat::kRtcpFIR:
      return "RtcpSpecific_FIR";
    case RtcpPayloadSpecificFormat::kRtcpREMB:
      return "RtcpSpecific_REMB";
    default:
      return tylib::format_string("Unknown[%d]", static_cast<int>(f));
  }
}

// if add new enum, modify ToString and rtcp handler switch case.
// should use polymorphism?
// https://datatracker.ietf.org/doc/html/rfc5760#section-5
// note: in RFC, "PT" in RTP is called "payload type", in RTCP is "packet type".
// per https://tools.ietf.org/html/rfc5761
enum class RtcpPacketType : uint8_t {
  RTCP_MIN_PT = 192,  // for FIR

  // basic:
  kSenderReport = 200,
  kReceiverReport = 201,
  kSourceDescription = 202,
  kBye = 203,
  kApplicationDefined = 204,

  // extend:
  kRtpFeedback = 205,
  kPayloadSpecificFeedback = 206,
  kExtendedReports = 207,

  RTCP_MAX_PT = 223,  // include
};

// https://github.com/wireshark/wireshark/blob/master/epan/dissectors/packet-rtcp.c#L147
inline std::string RtcpPacketTypeToString(RtcpPacketType type) {
  switch (type) {
    case RtcpPacketType::kSenderReport:
      return "SenderReport";
    case RtcpPacketType::kReceiverReport:
      return "ReceiverReport";
    case RtcpPacketType::kSourceDescription:
      return "SourceDescription";
    case RtcpPacketType::kBye:
      return "Bye";
    case RtcpPacketType::kApplicationDefined:
      return "ApplicationDefined";
    case RtcpPacketType::kRtpFeedback:
      return "RtpFeedback";
    case RtcpPacketType::kPayloadSpecificFeedback:
      return "PayloadSpecificFeedback";
    case RtcpPacketType::kExtendedReports:
      return "ExtendedReports";
    default:
      return "Unknowon[" + std::to_string(static_cast<int>(type)) + "]";
  }
}

// @brief check if RTCP,
// same as
// https://source.chromium.org/chromium/chromium/src/+/main:third_party/webrtc/modules/rtp_rtcp/source/rtp_util.cc;l=32;drc=184005f8792002e29052d653f4846121ee7d1f9a
// 128 + (64 <= payload_type && payload_type < 96), that is [192, 223]
//
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
inline bool isRtcp(RtcpPacketType packettype) {
  return packettype >= RtcpPacketType::RTCP_MIN_PT &&
         packettype <= RtcpPacketType::RTCP_MAX_PT;
}

// Generic NACK RTCP_RTP_FB + (FMT 1)rfc4585
//  0                   1                   2                   3
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |            PID                |             BLP               |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class NackBlock {
 private:
  uint32_t pid : 16;
  uint32_t blp : 16;

 public:
  uint16_t getNackPid() const { return ntohs(pid); }
  void setNackPid(uint16_t new_pid) { pid = htons(new_pid); }
  uint16_t getNackBlp() const { return ntohs(blp); }
  void setNackBlp(uint16_t new_blp) { blp = htons(new_blp); }
};

//  0                   1                   2                   3
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |V=2|P|    RC   |   PT=RR=201   |             length            | header
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                     SSRC of packet sender                     |
// +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
//
// RECEIVER REPORT
// |                 SSRC_1 (SSRC of first source)                 | report
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ block
// | fraction lost |       cumulative number of packets lost       |   1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |           extended highest sequence number received           |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                      interarrival jitter                      |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                         last SR (LSR)                         |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                   delay since last SR (DLSR)                  |
// +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
// |                 SSRC_2 (SSRC of second source)                | report
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ block
// :                               ...                             :   2
// +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
// |                  profile-specific extensions                  |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
// SENDER REPORT  // PT = 200
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |            NTP timestamp, most significant word NTS           |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |             NTP timestamp, least significant word             |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                       RTP timestamp RTS                       |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                   sender's packet count SPC                   |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                    sender's octet count SOC                   |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// 0                   1                   2                   3
// 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |V=2|P|    SC   |  PT=SDES=202  |             length            | header
// +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
// |                          SSRC/CSRC_1                          | chunk 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                           SDES items                          |
// |                              ...                              |
// +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
// |                          SSRC/CSRC_2                          | chunk 2
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                           SDES items                          |
// |                              ...                              |
// +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
//
//
// max = mantissa*2^exp
// from https://datatracker.ietf.org/doc/html/draft-alvestrand-rmcat-remb-03
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |V=2|P| FMT=15  |   PT=206      |             length            |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                  SSRC of packet sender                        |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                  SSRC of media source                         |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |  Unique identifier 'R' 'E' 'M' 'B'                            |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |  Num SSRC     | BR Exp    |  BR Mantissa                      |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |   SSRC feedback                                               |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |  ...                                                          |
//
// From RFC 3611: RTP Control Protocol Extended Reports (RTCP XR).
//
// Format for XR packets:
//
//  0                   1                   2                   3
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |V=2|P|reserved |   PT=XR=207   |             length            |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                              SSRC                             |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// :                         report blocks                         :
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
// XR block:
//  0                   1                   2                   3
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |  Block Type   |   reserved    |         block length          |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// :             type-specific block contents                      :
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// block length: 16 bits
//       The length of this report block, including the header, in 32-
//       bit words minus one.  If the block type definition permits,
//       zero is an acceptable value, signifying a block that consists
//       of only the BT, type-specific, and block length fields, with a
//       null type-specific block contents field.
class RtcpHeader {
 public:
  // maybe FMT for payload specific feedback
  uint32_t blockcount : 5;

  // padding (P): 1 bit
  //  If the padding bit is set, this individual RTCP packet contains
  //  some additional padding octets at the end which are not part of
  //  the control information but are included in the length field.  The
  //  last octet of the padding is a count of how many padding octets
  //  should be ignored, including itself (it will be a multiple of
  //  four).  Padding may be needed by some encryption algorithms with
  //  fixed block sizes.  In a compound RTCP packet, padding is only
  //  required on one individual packet because the compound packet is
  //  encrypted as a whole for the method in Section 9.1.  Thus, padding
  //  MUST only be added to the last individual packet, and if padding
  //  is added to that packet, the padding bit MUST be set only on that
  //  packet.  This convention aids the header validity checks described
  //  in Appendix A.2 and allows detection of packets from some early
  //  implementations that incorrectly set the padding bit on the first
  //  individual packet and add padding to the last individual packet.
  uint32_t padding : 1;
  uint32_t version : 2;

  RtcpPacketType packettype : 8;

  // length: 16 bits
  //  The length of this RTCP packet in 32-bit words minus one,
  //  including the header and any padding.  (The offset of one makes
  //  zero a valid length and avoids a possible infinite loop in
  //  scanning a compound RTCP packet, while counting 32-bit words
  //  avoids a validity check for a multiple of 4.)
  uint32_t length : 16;

  // The synchronization source identifier for the originator of this XR packet.
  uint32_t ssrc;

  union report_t {
    struct receiverReport_t {
      uint32_t ssrcsource;
      /* RECEIVER REPORT DATA*/
      uint32_t fractionlost : 8;
      int32_t lost : 24;
      uint32_t seqnumcycles : 16;
      uint32_t highestseqnum : 16;
      uint32_t jitter;
      uint32_t lastsr;
      uint32_t delaysincelast;  // DLSR
    } receiverReport;

    struct senderReport_t {
      uint64_t ntptimestamp;
      uint32_t rtprts;

      // sender's packet count: 32 bits
      // The total number of RTP data packets transmitted by the sender
      // since starting transmission up until the time this SR packet was
      // generated.  The count SHOULD be reset if the sender changes its
      // SSRC identifier.
      uint32_t packetsent;

      // sender's octet count: 32 bits
      // The total number of payload octets (i.e., not including header or
      // padding) transmitted in RTP data packets by the sender since
      // starting transmission up until the time this SR packet was
      // generated.  The count SHOULD be reset if the sender changes its
      // SSRC identifier.  This field can be used to estimate the average
      // payload data rate.
      uint32_t octetssent;
      struct receiverReport_t rrlist[1];
    } senderReport;

    struct sdes_t {
      uint32_t type : 8;
      uint32_t length : 8;
      char data[1];
    } sdes;

    struct genericNack_t {
      uint32_t ssrcsource;
      NackBlock nack_block;
    } nackPacket;

    struct remb_t {
      uint32_t ssrcsource;
      uint32_t uniqueid;
      uint32_t numssrc : 8;
      uint32_t brLength : 24;
      uint32_t ssrcfeedb[kSsrcFeedbackMaxNumber];
    } rembPacket;

    struct pli_t {
      uint32_t ssrcsource;
      uint32_t fci;
    } pli;

    struct fir_t {
      uint32_t ssrcsource;
      uint32_t mediasource;
      uint32_t seqnumber : 8;
      uint32_t reserved : 24;
    } fir;

    // Extended Reports (XR)
    // https://datatracker.ietf.org/doc/html/rfc3611
    struct XR_DLRR_t {
      EnXRBlockType blocktype : 8;
      uint32_t reserved : 8;

      // block length: 16 bits
      // The length of this report block, including the header, in 32-
      // bit words minus one.  If the block type definition permits,
      // zero is an acceptable value, signifying a block that consists
      // of only the BT, type-specific, and block length fields, with a
      // null type-specific block contents field.
      uint32_t blocklen : 16;

      // SSRC of nth receiver
      uint32_t rrssrc;

      // last RR timestamp (LRR): 32 bits
      // The middle 32 bits out of 64 in the NTP timestamp (as explained
      // in the previous section), received as part of a Receiver
      // Reference Time Report Block from participant SSRC_n.  If no
      // such block has been received, the field is set to zero.
      uint32_t lastrr;

      // delay since last RR (DLRR): 32 bits
      // The delay, expressed in units of 1/65536 seconds, between
      // receiving the last Receiver Reference Time Report Block from
      // participant SSRC_n and sending this DLRR Report Block.  If a
      // Receiver Reference Time Report Block has yet to be received
      // from SSRC_n, the DLRR field is set to zero (or the DLRR is
      // omitted entirely).  Let SSRC_r denote the receiver issuing this
      // DLRR Report Block.  Participant SSRC_n can compute the round-
      // trip propagation delay to SSRC_r by recording the time A when
      // this Receiver Timestamp Report Block is received.  It
      // calculates the total round-trip time A-LRR using the last RR
      // timestamp (LRR) field, and then subtracting this field to leave
      // the round-trip propagation delay as A-LRR-DLRR.  This is
      // illustrated in [9, Fig. 2].
      uint32_t dlrr;
    } xr_dlrr;

    struct XR_RRTR_t {
      EnXRBlockType blocktype : 8;
      uint32_t reserved : 8;
      uint32_t blocklen : 16;

      // NTP timestamp: 64 bits
      // Indicates the wallclock time when this block was sent so that
      // it may be used in combination with timestamps returned in DLRR
      // Report Blocks (see next section) from other receivers to
      // measure round-trip propagation to those receivers.  Receivers
      // should expect that the measurement accuracy of the timestamp
      // may be limited to far less than the resolution of the NTP
      // timestamp.  The measurement uncertainty of the timestamp is not
      // indicated as it may not be known.  A report block sender that
      // can keep track of elapsed time but has no notion of wallclock
      // time may use the elapsed time since joining the session
      // instead.  This is assumed to be less than 68 years, so the high
      // bit will be zero.  It is permissible to use the sampling clock
      // to estimate elapsed wallclock time.  A report sender that has
      // no notion of wallclock or elapsed time may set the NTP
      // timestamp to zero.
      uint32_t seconds;
      uint32_t fractions;
    } xr_rrtr;
  } report;

  RtcpHeader()
      : blockcount(0),
        padding(0),
        version(2),
        packettype(RtcpPacketType::RTCP_MIN_PT),
        length(0),
        ssrc(0) {}

  bool isFeedback(void) const {
    return (packettype == RtcpPacketType::kReceiverReport ||
            packettype == RtcpPacketType::kRtpFeedback ||
            packettype == RtcpPacketType::kPayloadSpecificFeedback);
  }

  RtcpPacketType getPacketType() const { return packettype; }
  void setPacketType(RtcpPacketType pt) { packettype = pt; }

  // maybe FMT for RtcpPayloadSpecificFormat or RtcpRtpFeedbackFormat
  uint8_t getBlockCount() const { return blockcount; }
  void setBlockCount(uint8_t count) { blockcount = count; }

  uint16_t getLength() const { return ntohs(length); }
  void setLength(uint16_t theLength) { length = htons(theLength); }
  int getRealLength() const { return (getLength() + 1) * 4; }

  uint32_t getSSRC() const { return ntohl(ssrc); }
  void setSSRC(uint32_t aSsrc) { ssrc = htonl(aSsrc); }

  // other type of rtcp first attr is also ssrcsource
  uint32_t getSourceSSRC() const {
    return ntohl(report.receiverReport.ssrcsource);
  }
  void setSourceSSRC(uint32_t sourceSsrc) {
    report.receiverReport.ssrcsource = htonl(sourceSsrc);
  }

  uint8_t getFractionLost() const { return report.receiverReport.fractionlost; }
  void setFractionLost(uint8_t fractionLost) {
    report.receiverReport.fractionlost = fractionLost;
  }

  uint32_t getLostPackets() const {
    return ntohl(report.receiverReport.lost) >> 8;
  }
  void setLostPackets(uint32_t lost) {
    report.receiverReport.lost = htonl(lost) >> 8;
  }

  uint16_t getSeqnumCycles() const {
    return ntohs(report.receiverReport.seqnumcycles);
  }
  void setSeqnumCycles(uint16_t seqnumcycles) {
    report.receiverReport.seqnumcycles = htons(seqnumcycles);
  }

  uint16_t getHighestSeqnum() const {
    return ntohs(report.receiverReport.highestseqnum);
  }
  void setHighestSeqnum(uint16_t highest) {
    report.receiverReport.highestseqnum = htons(highest);
  }

  uint32_t getJitter() const { return ntohl(report.receiverReport.jitter); }
  void setJitter(uint32_t jitter) {
    report.receiverReport.jitter = htonl(jitter);
  }

  uint32_t getLastSr() const { return ntohl(report.receiverReport.lastsr); }
  void setLastSr(uint32_t lastsr) {
    report.receiverReport.lastsr = htonl(lastsr);
  }

  uint32_t getDelaySinceLastSr() const {
    return ntohl(report.receiverReport.delaysincelast);
  }
  void setDelaySinceLastSr(uint32_t delaylastsr) {
    report.receiverReport.delaysincelast = htonl(delaylastsr);
  }

  uint32_t getPacketsSent() const {
    return ntohl(report.senderReport.packetsent);
  }
  void setPacketsSent(uint32_t packetssent) {
    report.senderReport.packetsent = htonl(packetssent);
  }

  uint32_t getOctetsSent() const {
    return ntohl(report.senderReport.octetssent);
  }
  void setOctetsSent(uint32_t octets_sent) {
    report.senderReport.octetssent = htonl(octets_sent);
  }

  uint64_t getNtpTimestamp() const {
    return (static_cast<uint64_t>(htonl(report.senderReport.ntptimestamp))
            << 32) +
           htonl(report.senderReport.ntptimestamp >> 32);
  }
  void setNtpTimestamp(uint64_t ntp_timestamp) {
    report.senderReport.ntptimestamp =
        (static_cast<uint64_t>(ntohl(ntp_timestamp)) << 32) +
        ntohl(ntp_timestamp >> 32);
  }

  uint32_t getRtpTimestamp() const { return ntohl(report.senderReport.rtprts); }
  void setRtpTimestamp(uint32_t rtp_timestamp) {
    report.senderReport.rtprts = htonl(rtp_timestamp);
  }

  uint32_t get32MiddleNtp() const {
    uint64_t middle = (report.senderReport.ntptimestamp << 16) >> 32;
    return ntohl(middle);
  }

  uint16_t getNackPid() const {
    return report.nackPacket.nack_block.getNackPid();
  }
  void setNackPid(uint16_t pid) {
    report.nackPacket.nack_block.setNackPid(pid);
  }

  uint16_t getNackBlp() const {
    return report.nackPacket.nack_block.getNackBlp();
  }
  void setNackBlp(uint16_t blp) {
    report.nackPacket.nack_block.setNackBlp(blp);
  }

  void setREMBBitRate(uint64_t bitRate) {
    uint64_t max = 0x3FFFF;  // 18 bits
    uint16_t exp = 0;

    while (bitRate >= max && exp < 64) {
      exp += 1;
      max = max << 1;
    }

    uint64_t mantissa = bitRate >> exp;
    exp = exp & 0x3F;
    mantissa = mantissa & 0x3FFFF;
    uint32_t line = mantissa + (exp << 18);
    report.rembPacket.brLength = htonl(line) >> 8;
  }
  uint64_t getREMBBitRate() const { return getBrMantis() << getBrExp(); }
  uint32_t getBrExp() const {
    // remove the 0s added by nothl (8) + the 18 bits of Mantissa
    return (ntohl(report.rembPacket.brLength) >> 26);
  }
  uint32_t getBrMantis() const {
    return (ntohl(report.rembPacket.brLength) >> 8 & 0x3ffff);
  }

  uint8_t getREMBNumSSRC() const { return report.rembPacket.numssrc; }
  void setREMBNumSSRC(uint8_t num) { report.rembPacket.numssrc = num; }

  uint32_t getREMBFeedSSRC(uint8_t index) const {
    return ntohl(report.rembPacket.ssrcfeedb[index]);
  }
  void setREMBFeedSSRC(uint8_t index, uint32_t ssrc) {
    report.rembPacket.ssrcfeedb[index] = htonl(ssrc);
  }

  uint32_t getFCI() const { return ntohl(report.pli.fci); }
  void setFCI(uint32_t fci) { report.pli.fci = htonl(fci); }

  void setFIRSourceSSRC(uint32_t ssrc) { report.fir.mediasource = htonl(ssrc); }
  void setFIRSequenceNumber(uint8_t seq_number) {
    report.fir.seqnumber = seq_number;
  }

  EnXRBlockType getBlockType() const { return report.xr_dlrr.blocktype; }
  void setBlockType(EnXRBlockType blockType) {
    report.xr_dlrr.blocktype = blockType;
  }

  uint16_t getBlockLen() const { return ntohs(report.xr_dlrr.blocklen); }
  void setBlockLen(uint16_t blockLen) {
    report.xr_dlrr.blocklen = htons(blockLen);
  }
  int getBlockRealLen() const { return (getBlockLen() + 1) * 4; }

  uint32_t getRrSsrc() const { return ntohl(report.xr_dlrr.rrssrc); }
  void setRrSsrc(uint32_t rrssrc) { report.xr_dlrr.rrssrc = htonl(rrssrc); }

  uint32_t getLastRr() const { return ntohl(report.xr_dlrr.lastrr); }
  void setLastRr(uint32_t lastrr) { report.xr_dlrr.lastrr = htonl(lastrr); }

  uint32_t getDLRR() const { return ntohl(report.xr_dlrr.dlrr); }
  void setDLRR(uint32_t dlrr) { report.xr_dlrr.dlrr = htonl(dlrr); }

  uint64_t getRrtrNtp() const {
    uint64_t seconds = ntohl(report.xr_rrtr.seconds);
    uint32_t fractions = ntohl(report.xr_rrtr.fractions);
    uint64_t ntp = (seconds << 32) | fractions;

    return ntp;
  }
  void setRrtrNtp(uint64_t ntp_timestamp) {
    report.xr_rrtr.seconds = htonl(ntp_timestamp >> 32);
    report.xr_rrtr.fractions = htonl(ntp_timestamp & 0xFFFFFFFF);
  }

  std::string ToString() const {
    return tylib::format_string(
        "{blockCnt=%d, pad=%d, packetType=%d[%s], len=%d(realLen=%d), "
        "senderSSRC=%u(0x%X)}",
        getBlockCount(), padding, static_cast<int>(getPacketType()),
        RtcpPacketTypeToString(getPacketType()).data(), getLength(),
        getRealLength(), getSSRC(), getSSRC());
  }
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PARSER_H_