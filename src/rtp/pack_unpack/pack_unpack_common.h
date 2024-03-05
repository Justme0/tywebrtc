
#ifndef RTP_PACK_UNPACK_PACK_UNPACK_COMMON_H_
#define RTP_PACK_UNPACK_PACK_UNPACK_COMMON_H_

#include "tylib/string/format_string.h"

// @see: https://tools.ietf.org/html/rfc6184#section-5.2

const int kH265StapA = 48;
const int kH265FuA = 49;

// H.264 nalu header type mask.
const int kH264TypeMask = 0x1F;

// H264协议中规定的nalu_unit_type所占的byte长度
const int VIDEO_NALU_HEADER_LTH = 1;
const int VIDEO_MAX_SPP_PPS_LEN = 100;

enum FuDefs { kSBit = 0x80, kEBit = 0x40, kRBit = 0x20 };

/*FU-A分片指示器结构 */
struct VideoFuIndicator {
  uint8_t type : 5;  // nalu的类型，RFC3984中FU-A为28，FU-B为29
  uint8_t
      nri : 2;    // 参考指示，0~3，这里对于单一nalu，将会使用2作为一组的数据包的结束
  uint8_t f : 1;  // 强制0位，无语法冲突为0，否则为1
};

/*FU-A分片头结构 */
struct VideoFuHeader {
  uint8_t type : 5;  // 分片nalu的payload类型
  uint8_t
      reserve : 1;  // 保留位，这里将会被用于标识是否是一组最后的一个数据包，若是则置1，否则为0
  uint8_t end : 1;  // 分片结束位，若为nalu的最后一个分片，置1，否则置0
  uint8_t start : 1;  // 分片开始位，若为nalu的第一个分片，置1，否则置0
};

#define VIDEO_FU_INDICATOR_SIZE \
  (sizeof(VideoFuIndicator))  // FU_indicator的字节长度
#define VIDEO_FU_HEADER_SIZE (sizeof(VideoFuHeader))  // FU_header的字节长度

// Nalu type定义，根据H264协议标准定义
// https://github.com/wireshark/wireshark/blob/master/epan/dissectors/packet-h264.c#L302
enum enVideoH264NaluType {
  kVideoNaluUnspecific = 0,
  kVideoNaluSlice = 1,
  kVideoNaluDpa = 2,
  kVideoNaluDpb = 3,
  kVideoNaluDpc = 4,
  kVideoNaluIdr = 5,
  kVideoNaluSei = 6,
  kVideoNaluSps = 7,
  kVideoNaluPps = 8,
  kVideoNaluDelimiterRbsp = 9,
  kVideoNaluEoseq = 10,
  kVideoNaluEostm = 11,
  kVideoNaluFillerData = 12,
  kVideoNaluSpsExtn = 13,
  kVideoNaluPrefix = 14,
  kVideoNaluSubSps = 15,
  kVideoNaluSliceAux = 19,
  kVideoNaluSliceExt = 20,
  kVideoNaluSliceExtDepth = 21,
  // https://datatracker.ietf.org/doc/html/rfc6184#section-5.2
  kVideoNaluStapA = 24,
  kVideoNaluStapB = 25,
  kVideoNaluMtap16 = 26,
  kVideoNaluMtap24 = 27,
  kVideoNaluFuA = 28,
  kVideoNaluFuB = 29,
  kVideoNaluPacsi = 30,
  kVideoNaluExt = 31,
};

inline std::string enVideoH264NaluTypeToString(enVideoH264NaluType v) {
  switch (v) {
    case kVideoNaluUnspecific:
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
    case kVideoNaluDelimiterRbsp:
      return "NAL unit - Access unit delimiter";
    case kVideoNaluEoseq:
      return "NAL unit - End of sequence";
    case kVideoNaluEostm:
      return "NAL unit - End of stream";
    case kVideoNaluFillerData:
      return "NAL unit - Filler data";
    case kVideoNaluSpsExtn:
      return "NAL unit - Sequence parameter set extension";
    case kVideoNaluPrefix:
      return "NAL unit - Prefix";
    case kVideoNaluSubSps:
      return "NAL unit - Subset sequence parameter set";
    // "NAL unit - Reserved" // 16
    // "NAL unit - Reserved" // 17
    // "NAL unit - Reserved" // 18
    case kVideoNaluSliceAux:
      return "NAL unit - Coded slice of an auxiliary coded picture without "
             "partitioning";
    case kVideoNaluSliceExt:
      return "NAL unit - Coded slice extension";
    case kVideoNaluSliceExtDepth:
      return "NAL unit - Coded slice extension for depth view components";
    // "NAL unit - Reserved" // 22
    // "NAL unit - Reserved" // 23
    case kVideoNaluStapA:
      return "Single-time aggregation packet A (STAP-A)";
    case kVideoNaluStapB:
      return "Single-time aggregation packet B (STAP-B)";
    case kVideoNaluMtap16:
      return "Multi-time aggregation packet 16 (MTAP16)";
    case kVideoNaluMtap24:
      return "Multi-time aggregation packet 24 (MTAP24)";
    case kVideoNaluFuA:
      return "Fragmentation unit A (FU-A)";
    case kVideoNaluFuB:
      return "Fragmentation unit B (FU-B)";
    case kVideoNaluPacsi:
      return "NAL unit - Payload Content Scalability Information (PACSI)";
    case kVideoNaluExt:
      return "NAL unit - Extended NAL Header";
    default:
      return tylib::format_string("Unknown[%d]", v);
  }
}

#endif  //  RTP_PACK_UNPACK_PACK_UNPACK_COMMON_H_