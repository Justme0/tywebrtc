// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_PACK_UNPACK_H264_COMMON_H_
#define SRC_RTP_PACK_UNPACK_H264_COMMON_H_

#include "tylib/string/format_string.h"

namespace tywebrtc {

// H264协议中规定的nalu_unit_type所占的byte长度
const int VIDEO_NALU_HEADER_LTH = 1;
const int VIDEO_MAX_SPP_PPS_LEN = 100;

enum FuDefs { kSBit = 0x80, kEBit = 0x40, kRBit = 0x20 };

/*FU-A分片指示器结构 */
struct VideoFuIndicator {
  // nalu的类型，RFC3984中FU-A为28，FU-B为29
  uint8_t type : 5;
  // 参考指示，0~3，这里对于单一nalu，将会使用2作为一组的数据包的结束
  uint8_t nri : 2;
  // 强制0位，无语法冲突为0，否则为1
  uint8_t f : 1;
};

/*FU-A分片头结构 */
struct VideoFuHeader {
  // 分片nalu的payload类型
  uint8_t type : 5;
  // 保留位，这里将会被用于标识是否是一组最后的一个数据包，若是则置1，否则为0
  uint8_t reserve : 1;
  // 分片结束位，若为nalu的最后一个分片，置1，否则置0
  uint8_t end : 1;
  // 分片开始位，若为nalu的第一个分片，置1，否则置0
  uint8_t start : 1;
};

// FU_indicator的字节长度
const int VIDEO_FU_INDICATOR_SIZE = sizeof(VideoFuIndicator);
// FU_header的字节长度
const int VIDEO_FU_HEADER_SIZE = sizeof(VideoFuHeader);

// 史上最全 H.264 NALU type 定义
// https://github.com/wireshark/wireshark/blob/master/epan/dissectors/packet-h264.c#L302
enum EnVideoH264NaluType {
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
  kVideoNaluDps = 16,
  kVideoNaluReserved17 = 17,
  kVideoNaluReserved18 = 18,
  kVideoNaluSliceAux = 19,
  kVideoNaluSliceExt = 20,
  kVideoNaluSliceExtDepth = 21,
  kVideoNaluReserved22 = 22,
  kVideoNaluReserved23 = 23,
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

inline std::string EnVideoH264NaluTypeToString(EnVideoH264NaluType v) {
  switch (v) {
    case kVideoNaluUnspecific:
      return "Undefined";
    case kVideoNaluSlice:
      return "Coded slice of a non-IDR picture";
    case kVideoNaluDpa:
      return "Coded slice data partition A";
    case kVideoNaluDpb:
      return "Coded slice data partition B";
    case kVideoNaluDpc:
      return "Coded slice data partition C";
    case kVideoNaluIdr:
      return "Coded slice of an IDR picture";
    case kVideoNaluSei:
      return "Supplemental enhancement information (SEI)";
    case kVideoNaluSps:
      return "Sequence parameter set";
    case kVideoNaluPps:
      return "Picture parameter set";
    case kVideoNaluDelimiterRbsp:
      return "Access unit delimiter";
    case kVideoNaluEoseq:
      return "End of sequence";
    case kVideoNaluEostm:
      return "End of stream";
    case kVideoNaluFillerData:
      return "Filler data";
    case kVideoNaluSpsExtn:
      return "Sequence parameter set extension";
    case kVideoNaluPrefix:
      return "Prefix";
    case kVideoNaluSubSps:
      return "Subset sequence parameter set";
    case kVideoNaluDps:
      return "Depth parameter set";
    case kVideoNaluReserved17:
      return "Reserved 17";
    case kVideoNaluReserved18:
      return "Reserved 18";
    case kVideoNaluSliceAux:
      return "Coded slice of an auxiliary coded picture without "
             "partitioning";
    case kVideoNaluSliceExt:
      return "Coded slice extension";
    case kVideoNaluSliceExtDepth:
      return "Coded slice extension for depth view components";
    case kVideoNaluReserved22:
      return "Reserved 22";
    case kVideoNaluReserved23:
      return "Reserved 23";
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
      return "Payload Content Scalability Information (PACSI)";
    case kVideoNaluExt:
      return "Extended NAL Header";
    default:
      return tylib::format_string("Unknown[%d]", v);
  }
}

// H.264 nalu header type mask.
const int kH264TypeMask = 0x1F;
inline EnVideoH264NaluType GetNaluType(const char* nalu) {
  return static_cast<EnVideoH264NaluType>(nalu[0] & kH264TypeMask);
}

}  // namespace tywebrtc

#endif  //  SRC_RTP_PACK_UNPACK_H264_COMMON_H_