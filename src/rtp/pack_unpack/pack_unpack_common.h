
#ifndef RTP_PACK_UNPACK_PACK_UNPACK_COMMON_H_
#define RTP_PACK_UNPACK_PACK_UNPACK_COMMON_H_

#include "tylib/string/format_string.h"

// @see: https://tools.ietf.org/html/rfc6184#section-5.2

const int kH265StapA = 48;
const int kH265FuA = 49;

// move to head file

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
  kVideoNakuUnspecific = 0,
  kVideoNaluSlice = 1,
  kVideoNaluDpa = 2,
  kVideoNaluDpb = 3,
  kVideoNaluDpc = 4,
  kVideoNaluIdr = 5,
  kVideoNaluSei = 6,
  kVideoNaluSps = 7,
  kVideoNaluPps = 8,
  kVideoNaluUnitDelimiterRbsp = 9,
  kVideoNaluUnitEoseq = 10,
  kVideoNaluUnitEostm = 11,
  kVideoNaluUnitFilter = 12,
  kVideoNaluUnitSpsExtn = 13,
  kH264StapA = 24,
  kH264FuA = 28,
};

inline std::string enVideoH264NaluTypeToString(enVideoH264NaluType v) {
  switch (v) {
    case kVideoNakuUnspecific:
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
    case kVideoNaluUnitDelimiterRbsp:
      return "NAL unit - Access unit delimiter";
    case kVideoNaluUnitEoseq:
      return "NAL unit - End of sequence";
    case kVideoNaluUnitEostm:
      return "NAL unit - End of stream";
    case kVideoNaluUnitFilter:
      return "NAL unit - Filler data";
    case kVideoNaluUnitSpsExtn:
      return "NAL unit - Sequence parameter set extension";
    // "NAL unit - Prefix" // 14
    // "NAL unit - Subset sequence parameter set" // 15
    // "NAL unit - Reserved" // 16
    // "NAL unit - Reserved" // 17
    // "NAL unit - Reserved" // 18
    // "NAL unit - Coded slice of an auxiliary coded picture without
    // partitioning" // 19
    // "NAL unit - Coded slice extension" // 20
    // "NAL unit - Coded slice extension for depth view components" // 21
    // "NAL unit - Reserved" // 22
    // "NAL unit - Reserved" // 23
    case kH264StapA:
      return "Single-time aggregation packet A (STAP-A)";
    //  "Single-time aggregation packet B (STAP-B)" // 25
    // "Multi-time aggregation packet 16 (MTAP16)" // 26
    // "Multi-time aggregation packet 24 (MTAP24)" // 27
    case kH264FuA:
      return "Fragmentation unit A (FU-A)";
    // "Fragmentation unit B (FU-B)" // 29
    // "NAL unit - Payload Content Scalability Information (PACSI)" // 30
    // "NAL unit - Extended NAL Header" // 31
    default:
      return tylib::format_string("Unknown[%d]", v);
  }
}

#endif  //  RTP_PACK_UNPACK_PACK_UNPACK_COMMON_H_