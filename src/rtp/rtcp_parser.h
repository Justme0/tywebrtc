// from licode

#ifndef RTP_RTCP_PARSER_H_
#define RTP_RTCP_PARSER_H_

#include <netinet/in.h>

#include "log/log.h"
#include "tylib/string/format_string.h"

// RTCP Payload types
enum {
  RTCP_MIN_PT = 194,       // per https://tools.ietf.org/html/rfc5761
  RTCP_Sender_PT = 200,    // RTCP Sender Report
  RTCP_Receiver_PT = 201,  // RTCP Receiver Report
  RTCP_SDES_PT = 202,
  RTCP_BYE = 203,
  RTCP_APP = 204,
  RTCP_RTP_Feedback_PT = 205,  // RTCP Transport Layer Feedback Packet
  RTCP_PS_Feedback_PT = 206,   // RTCP Payload Specific Feedback Packet
  RTCP_XR_PT = 207,            // rfc3611
  RTCP_MAX_PT = 223,
};

#define RRTR_BT 4
#define DLRR_BT 5

// RTCP_PS_Feedback_PT SubType
#define RTCP_PLI_FMT 1
#define RTCP_SLI_FMT 2
#define RTCP_FIR_FMT 4
#define RTCP_AFB 15

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
  uint32_t blockcount : 5;
  uint32_t padding : 1;
  uint32_t version : 2;
  uint32_t packettype : 8;
  uint32_t length : 16;
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
      uint32_t ssrcsource;
      NackBlock nack_block;
    } nackPacket;

    struct remb_t {
      uint32_t ssrcsource;
      uint32_t uniqueid;
      uint32_t numssrc : 8;
      uint32_t brLength : 24;
      uint32_t ssrcfeedb;
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
        packettype(0),
        length(0),
        ssrc(0) {}

  bool isFeedback(void) {
    return (packettype == RTCP_Receiver_PT ||
            packettype == RTCP_PS_Feedback_PT ||
            packettype == RTCP_RTP_Feedback_PT);
  }

  // no use?
  bool isRtcp(void) {
    return (packettype >= RTCP_MIN_PT && packettype <= RTCP_MAX_PT);
  }
  uint8_t getPacketType() const { return packettype; }
  void setPacketType(uint8_t pt) { packettype = pt; }
  uint8_t getBlockCount() const { return (uint8_t)blockcount; }
  void setBlockCount(uint8_t count) { blockcount = count; }
  uint16_t getLength() const { return ntohs(length); }
  void setLength(uint16_t theLength) { length = htons(theLength); }
  uint32_t getSSRC() const { return ntohl(ssrc); }
  void setSSRC(uint32_t aSsrc) { ssrc = htonl(aSsrc); }
  uint32_t getSourceSSRC() const {
    return ntohl(report.receiverReport.ssrcsource);
  }
  void setSourceSSRC(uint32_t sourceSsrc) {
    report.receiverReport.ssrcsource = htonl(sourceSsrc);
  }
  uint8_t getFractionLost() const {
    return (uint8_t)report.receiverReport.fractionlost;
  }
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
    return (((uint64_t)htonl(report.senderReport.ntptimestamp)) << 32) +
           htonl(report.senderReport.ntptimestamp >> 32);
  }
  void setNtpTimestamp(uint64_t ntp_timestamp) {
    report.senderReport.ntptimestamp =
        (((uint64_t)ntohl(ntp_timestamp)) << 32) + ntohl(ntp_timestamp >> 32);
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
    uint32_t seconds = ntohl(report.xr_rrtr.seconds);
    uint32_t fractions = ntohl(report.xr_rrtr.fractions);

    ntp = (((uint64_t)seconds) << 32) | fractions;

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
        "{blockCnt=%d, pad=%d, packetType=%d, size=%d, ssrc=%d}",
        getBlockCount(), padding, getPacketType(), getLength(), getSSRC());
  }
};

#endif  // RTP_RTCP_PARSER_H_