// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// https://datatracker.ietf.org/doc/html/rfc3550#section-6
// From licode
// https://github.com/lynckia/licode/blob/master/erizo/src/erizo/rtp/RtpHeaders.h

#ifndef SRC_RTP_RTCP_RTCP_PARSER_H_
#define SRC_RTP_RTCP_RTCP_PARSER_H_

#include <netinet/in.h>

#include <cstdint>

#include "tylib/string/format_string.h"

#include "src/log/log.h"

namespace tywebrtc {

#define RRTR_BT 4
#define DLRR_BT 5

// payload type is 205
enum class RtcpGenericFeedbackFormat {
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
  kSenderReport = 200,
  kReceiverReport = 201,
  kSourceDescription = 202,
  kBye = 203,
  kApplicationDefined = 204,
  kGenericRtpFeedback = 205,  // NACK RFC3550
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
    case RtcpPacketType::kGenericRtpFeedback:
      return "GenericRtpFeedback";
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
//
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
// |  Num SSRC     | BR Exp    |  BR Mantissa                      | max=
// mantissa*2^exp
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
// Extended report block:
//  0                   1                   2                   3
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |  Block Type   |   reserved    |         block length          |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// :             type-specific block contents                      :
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class RtcpHeader {
 public:
  // maybe FMT for payload specific feedback
  uint32_t blockcount : 5;
  uint32_t padding : 1;
  uint32_t version : 2;

  RtcpPacketType packettype : 8;
  uint32_t length : 16;

  uint32_t ssrc;

  union report_t {
    struct receiverReport_t {
      uint32_t mediaSourceSSRC;
      /* RECEIVER REPORT DATA*/
      uint32_t fractionlost : 8;
      int32_t lost : 24;
      uint32_t seqnumcycles : 16;
      uint32_t highestseqnum : 16;
      uint32_t jitter;
      uint32_t lastsr;
      uint32_t delaysincelast;
    } receiverReport;

    struct senderReport_t {
      uint64_t ntptimestamp;
      uint32_t rtprts;
      uint32_t packetsent;
      uint32_t octetssent;
      struct receiverReport_t rrlist[1];
    } senderReport;

    struct genericNack_t {
      uint32_t mediaSourceSSRC;
      NackBlock nack_block;
    } nackPacket;

    struct remb_t {
      uint32_t mediaSourceSSRC;
      uint32_t uniqueid;
      uint32_t numssrc : 8;
      uint32_t brLength : 24;
      uint32_t ssrcfeedb;
    } rembPacket;

    struct pli_t {
      uint32_t mediaSourceSSRC;
      uint32_t fci;
    } pli;

    struct fir_t {
      uint32_t mediaSourceSSRC;
      uint32_t mediasource;
      uint32_t seqnumber : 8;
      uint32_t reserved : 24;
    } fir;

    // Extended Reports (XR)
    // https://datatracker.ietf.org/doc/html/rfc3611
    struct XR_DLRR_t {
      uint32_t blocktype : 8;
      uint32_t reserved : 8;
      uint32_t blocklen : 16;
      uint32_t rrssrc;
      uint32_t lastrr;
      uint32_t dlrr;
    } xr_dlrr;

    struct XR_RRTR_t {
      uint32_t blocktype : 8;
      uint32_t reserved : 8;
      uint32_t blocklen : 16;
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
            packettype == RtcpPacketType::kGenericRtpFeedback ||
            packettype == RtcpPacketType::kPayloadSpecificFeedback);
  }

  RtcpPacketType getPacketType() const { return packettype; }
  void setPacketType(RtcpPacketType pt) { packettype = pt; }

  // maybe FMT for RtcpPayloadSpecificFormat or RtcpGenericFeedbackFormat
  uint8_t getBlockCount() const { return blockcount; }
  void setBlockCount(uint8_t count) { blockcount = count; }

  uint16_t getLength() const { return ntohs(length); }
  void setLength(uint16_t theLength) { length = htons(theLength); }

  uint32_t getSSRC() const { return ntohl(ssrc); }
  void setSSRC(uint32_t aSsrc) { ssrc = htonl(aSsrc); }

  uint32_t getMediaSourceSSRC() const {
    return ntohl(report.receiverReport.mediaSourceSSRC);
  }
  void setMediaSourceSSRC(uint32_t mediaSourceSSRC) {
    report.receiverReport.mediaSourceSSRC = htonl(mediaSourceSSRC);
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

  uint32_t getREMBFeedSSRC() const {
    return ntohl(report.rembPacket.ssrcfeedb);
  }
  void setREMBFeedSSRC(uint32_t ssrc) {
    report.rembPacket.ssrcfeedb = htonl(ssrc);
  }

  uint32_t getFCI() const { return ntohl(report.pli.fci); }
  void setFCI(uint32_t fci) { report.pli.fci = htonl(fci); }

  void setFIRSourceSSRC(uint32_t ssrc) { report.fir.mediasource = htonl(ssrc); }
  void setFIRSequenceNumber(uint8_t seq_number) {
    report.fir.seqnumber = seq_number;
  }

  uint8_t getBlockType() const { return report.xr_dlrr.blocktype; }
  void setBlockType(uint8_t blockType) { report.xr_dlrr.blocktype = blockType; }

  uint16_t getBlockLen() const { return ntohs(report.xr_dlrr.blocklen); }
  void setBlockLen(uint16_t blockLen) {
    report.xr_dlrr.blocklen = htons(blockLen);
  }

  uint32_t getRrSsrc() const { return ntohl(report.xr_dlrr.rrssrc); }
  void setRrSsrc(uint32_t rrssrc) { report.xr_dlrr.rrssrc = htonl(rrssrc); }

  uint32_t getLastRr() const { return ntohl(report.xr_dlrr.lastrr); }
  void setLastRr(uint32_t lastrr) { report.xr_dlrr.lastrr = htonl(lastrr); }

  uint32_t getDLRR() const { return ntohl(report.xr_dlrr.dlrr); }
  void setDLRR(uint32_t dlrr) { report.xr_dlrr.dlrr = htonl(dlrr); }

  uint64_t getRrtrNtp() const {
    // return (((uint64_t)htonl(report.xr_rrtr.ntp)) << 32) +
    // htonl(report.xr_rrtr.ntp >> 32);
    uint64_t ntp = 0;
    uint64_t seconds = ntohl(report.xr_rrtr.seconds);
    uint32_t fractions = ntohl(report.xr_rrtr.fractions);

    ntp = (seconds << 32) | fractions;

    return ntp;
  }
  void setRrtrNtp(uint64_t ntp_timestamp) {
    // report.xr_rrtr.ntp = (((uint64_t)ntohl(ntp_timestamp)) << 32) +
    // ntohl(ntp_timestamp >> 32);
    report.xr_rrtr.seconds = htonl(ntp_timestamp >> 32);
    report.xr_rrtr.fractions = htonl(ntp_timestamp & 0xFFFFFFFF);
  }

  std::string ToString() const {
    return tylib::format_string(
        "{blockCnt=%d, pad=%d, packetType=%d[%s], len=%d, senderSSRC=%u(0x%X)}",
        getBlockCount(), padding, static_cast<int>(getPacketType()),
        RtcpPacketTypeToString(getPacketType()).data(), getLength(), getSSRC(),
        getSSRC());
  }
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PARSER_H_