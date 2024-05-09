#include "src/rtp/pack_unpack/rtp_to_opus.h"

#include "src/rtp/rtp_parser.h"

// #include "webrtc_helper.h"
// #include "media_relay.h"
// #include "common/dump_webrtc.h"
namespace tywebrtc {

RtpDepacketizerOpus::RtpDepacketizerOpus() {}

RtpDepacketizerOpus::RtpDepacketizerOpus(uint8_t audio_level_index) {
  audio_level_index_ = audio_level_index;
}

int RtpDepacketizerOpus::Process(uint8_t* data, uint32_t length) {
  return ProcessExtInfo(data, length);
}

void RtpDepacketizerOpus::DecodeOneByteExt(const uint8_t* pStart,
                                           const uint8_t* pExtEnd) {
  if ((NULL == pStart) || (NULL == pExtEnd)) {
    return;
  }

  while (pExtEnd - pStart > 0) {
    RTP_EXT_COMMON_HRAD* pExtCommHead = (RTP_EXT_COMMON_HRAD*)pStart;
    // 当前扩展的长度是byte作为单位，长度范围是0-16，注意0表示一个字节
    const int type = pExtCommHead->ExtId;
    const int len = pExtCommHead->ExtLen;
    pStart++;

    // 跳过padding数据
    if (0 == type) {
      continue;
    }

#define MAX_RTP_EXT_TYPE 15

    if (MAX_RTP_EXT_TYPE == type) {
      tylog("RTP extension header 15 encountered. Terminate parsing");
      return;
    }

    if (pExtEnd - pStart < (len + 1)) {
      tylog("Incorrect one-byte extension len:%d bytes left in buffer:%lu",
            (len + 1), (pExtEnd - pStart));
      return;
    }

    if (type == audio_level_index_) {
      if (0 != len) {  // 0 means 1byte
        tylog("Incorrect RTP_EXT_VIDEO_ROTATION len: %d", len);
        return;
      }

      ST_RTP_EXT_AUDIO_LEVEL* pRtpExtVideoRotation =
          reinterpret_cast<ST_RTP_EXT_AUDIO_LEVEL*>(pExtCommHead);
      audio_level_ = pRtpExtVideoRotation->Level;
    }

    pStart += (len + 1);
  }
}

int RtpDepacketizerOpus::ProcessExtInfo(uint8_t* data, int length) {
  RtpHeader* header = reinterpret_cast<RtpHeader*>(data);

  if (header->getHeaderLength() > length) {
    return -1;
  }

  /*如果x=1则包含 4个字节类型 2个字节扩展长度(4个字节作为单位) */
  if (header->getExtension()) {
    const uint16_t ext_length = header->getHeaderExt()->getExtLength();

#define VIDEO_RFC5285_ONEBYTE_ORDER_NAME (0xBEDE)  // RFC5285 one-byte 名字

    if (VIDEO_RFC5285_ONEBYTE_ORDER_NAME ==
        header->getHeaderExt()->getExtId()) {
      // self head (payload + length) 4B
      const uint8_t* ext_start =
          reinterpret_cast<const uint8_t*>(header->getHeaderExt()) + 4;

      DecodeOneByteExt(ext_start, ext_start + ext_length * 4);
    }
  }

  return 0;
}

int RtpDepacketizerOpus::ParseAudioLevel(uint8_t* data, uint32_t length,
                                         uint32_t& audio_level) {
  if (ProcessExtInfo(data, length) < 0) {
    return -1;
  }
  audio_level = audio_level_;
  return 0;
}
}