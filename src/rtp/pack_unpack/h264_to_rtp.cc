#include "rtp/pack_unpack/h264_to_rtp.h"

#include <cassert>

#include "log/log.h"
#include "rtp/pack_unpack/pack_unpack_common.h"
#include "rtp/rtp_parser.h"

// @see: https://tools.ietf.org/html/rfc6184#section-5.8

// to refactor, use string/string_view, cancle the struct
struct Nalu {
  const char* data;
  int size;
};

// parse raw stream to several NALUs
static int ParseFrameNalus(const std::vector<char>& stream,
                           std::vector<Nalu>& nalus) {
  const char* frame = stream.data();
  int len = stream.size();

  // OPT magic number
  if (len < 4) {
    return -1;
  }

  if (frame[0] == 0x00 && frame[1] == 0x00 && frame[2] == 0x00 &&
      frame[3] == 0x01) {
    // Annex-B
    // 00 00 00 01 ab cd ef 00 00 00 01
    int pos = 0;
    const char* nal = nullptr;
    while (pos + 3 < len) {
      if (frame[pos] != 0x00 || frame[pos + 1] != 0x00 ||
          frame[pos + 2] != 0x01) {
        ++pos;
        continue;
      }

      if (nal != nullptr) {
        Nalu nalu;
        nalu.data = nal;
        nalu.size = frame + pos - nal;
        if (pos >= 1 && frame[pos - 1] == 0x00) {
          nalu.size -= 1;
        }
        nalus.push_back(nalu);
      }

      pos += 3;

      nal = frame + pos;
    }

    if (nal != nullptr) {
      Nalu nalu;
      nalu.data = nal;
      nalu.size = frame + len - nal;
      if (nalu.size < 0) {
        return -1;
      }
      nalus.push_back(nalu);
    }
  } else {
    // AVCC
    int pos = 0;
    while (pos < len) {
      int nal_len = (frame[pos + 0] << 24) | (frame[pos + 1] << 16) |
                    (frame[pos + 2] << 8) | frame[pos + 3];
      pos += 4;
      if (pos + nal_len > len) {
        return -1;
      }

      Nalu nalu;
      nalu.data = frame + pos;
      nalu.size = nal_len;
      if (nalu.size < 0) {
        return -1;
      }

      nalus.push_back(nalu);

      pos += nal_len;
    }
  }

  return 0;
}

int H264Packetizer::PacketStapA(const uint32_t timestamp,
                                const std::vector<std::shared_ptr<Extension>>&,
                                std::vector<RtpBizPacket>& rtpBizPackets) {
  if (sps_.empty() || pps_.empty()) {
    return 0;
  }

  uint8_t header = sps_[0];
  uint8_t nal_type = header & kH264TypeMask;

  std::vector<char> packet(MAX_PKT_BUF_SIZE);
  RtpHeader& rtp_header = *reinterpret_cast<RtpHeader*>(packet.data());

  // std::unique_ptr<RtpHeader> packet = RtpHeader::Create(MAX_PKT_BUF_SIZE);
  // RtpPacketImpl* pkt = static_cast<RtpPacketImpl*>(packet.get());
  // auto dst = pkt->data->data();
  rtp_header.setVersion(2);
  rtp_header.setPadding(0);
  rtp_header.setExtension(0);  // may have taylor, and add ext
  rtp_header.setCc(0);

  rtp_header.setMarker(0);  // last's mark setted outside
  rtp_header.setPayloadType(payload_type_);

  rtp_header.setSeqNumber(GeneratePowerSequence());
  rtp_header.setTimestamp(timestamp);
  rtp_header.setSSRC(ssrc_);

  char* dst = packet.data() + rtp_header.getHeaderLength();

  // stap-a header
  uint8_t stap_a_header = kVideoNaluStapA;
  stap_a_header |= (nal_type & (~kH264TypeMask));
  *dst++ = stap_a_header;

  WriteBigEndian(dst, sps_.size(), 2);
  dst += 2;
  memcpy(dst, sps_.data(), sps_.size());
  dst += sps_.size();
  WriteBigEndian(dst, pps_.size(), 2);
  dst += 2;
  memcpy(dst, pps_.data(), pps_.size());
  dst += pps_.size();

  packet.resize(dst - packet.data());

  RtpBizPacket rtpBizPacket(std::move(packet),
                            SplitPowerSeq(powerSequence_).first);
  rtpBizPacket.enterJitterTimeMs = g_now_ms;
  rtpBizPackets.emplace_back(std::move(rtpBizPacket));

  return 0;
}

int H264Packetizer::PacketSingleNalu(
    const char* frame, const int len, const uint32_t timestamp,
    const std::vector<std::shared_ptr<Extension>>& extensions,
    std::vector<RtpBizPacket>& rtpBizPackets) {
  if ((frame[0] & kH264TypeMask) == 0x07) {
    sps_.assign(reinterpret_cast<const char*>(frame), len);
    pps_.clear();
    // return 0;
  }

  if ((frame[0] & kH264TypeMask) == 0x08) {
    pps_.append(reinterpret_cast<const char*>(frame), len);
    // why not using assign?
    // pps_.assign(reinterpret_cast<const char*>(frame), len);
    // return 0;
  }

  if ((frame[0] & kH264TypeMask) == 0x05) {
    PacketStapA(timestamp, extensions, rtpBizPackets);
  }

  std::vector<char> packet(MAX_PKT_BUF_SIZE);
  RtpHeader& header = *reinterpret_cast<RtpHeader*>(packet.data());
  header.setVersion(2);  // fix number
  header.setTimestamp(timestamp);
  header.setSSRC(ssrc_);
  header.setSeqNumber(GeneratePowerSequence());
  header.setPayloadType(payload_type_);
  header.setMarker(0);
  header.setExtension(0);  // taylor

  const int headLen = header.getHeaderLength();
  char* ptr = packet.data() + headLen;
  memcpy(ptr, frame, len);

  packet.resize(headLen + len);

  RtpBizPacket rtpBizPacket(std::move(packet),
                            SplitPowerSeq(powerSequence_).first);
  rtpBizPacket.enterJitterTimeMs = g_now_ms;
  rtpBizPackets.emplace_back(std::move(rtpBizPacket));

  return 0;
}

int H264Packetizer::PacketFuA(
    const char* frame, const int len, const uint32_t timestamp,
    const std::vector<std::shared_ptr<Extension>>& extensions,
    std::vector<RtpBizPacket>& rtpBizPackets) {
  assert(len > kGuessMtuByte);
  // WEBRTC_ERROR_CHECK(stream_id_.c_str(), (nullptr == frame), 0,
  //                    "Chn %" PRIu64 " rtp buf is nullptr!", ssrc_);
  // WEBRTC_ERROR_CHECK(stream_id_.c_str(), (VIDEO_NALU_HEADER_LTH >= len), 0,
  //                    "Chn %" PRIu64 " rtp data len %d error.", ssrc_, len);

  if ((frame[0] & kH264TypeMask) == 0x05) {
    PacketStapA(timestamp, extensions, rtpBizPackets);
  }

  int32_t data_len = len - VIDEO_NALU_HEADER_LTH;
  assert(data_len >= kGuessMtuByte);

  const int RTP_HEADER_LTH = 12;
  int32_t slice_len = kGuessMtuByte - RTP_HEADER_LTH;

  int32_t pkt_num = data_len / slice_len;

  assert(pkt_num >= 1);

  int32_t last_pkt_len = data_len - pkt_num * slice_len;
  pkt_num += ((0 < last_pkt_len) ? 1 : 0);

  slice_len = (data_len + (pkt_num >> 1)) / pkt_num;

  const char* nalu = frame + VIDEO_NALU_HEADER_LTH;
  uint8_t fu_indicator = (frame[0] & 0xE0) | 28;  // FU-A
  uint8_t fu_header = frame[0] & 0x1F;
  int32_t pkt_cnt = 0;
  int32_t i = 0;
  for (fu_header |= kSBit; i < pkt_num; i++) {
    int32_t copy_len = 0;

    if (i < (pkt_num - 1)) {
      copy_len = slice_len;
    } else {
      copy_len = (data_len - (pkt_cnt * slice_len));
    }

    std::vector<char> packet(MAX_PKT_BUF_SIZE);
    RtpHeader& header = *reinterpret_cast<RtpHeader*>(packet.data());
    header.setVersion(2);
    header.setTimestamp(timestamp);
    header.setSSRC(ssrc_);
    header.setSeqNumber(GeneratePowerSequence());
    header.setPayloadType(payload_type_);
    header.setMarker(0);
    // header.extensions = extensions;

    int headerLen = header.getHeaderLength();
    int length =
        headerLen + VIDEO_FU_INDICATOR_SIZE + VIDEO_FU_HEADER_SIZE + copy_len;
    packet.resize(
        length);  // taylor check if out of bound, OPT: use append style

    char* cur_pos = packet.data() + headerLen;
    if ((pkt_num - 1) == i) {
      fu_header = kEBit | (fu_header & 0x1F);  // FU-A end
    }
    *cur_pos++ = fu_indicator;
    *cur_pos++ = fu_header;

    memcpy(cur_pos, nalu, copy_len);

    RtpBizPacket rtpBizPacket(std::move(packet),
                              SplitPowerSeq(powerSequence_).first);
    rtpBizPacket.enterJitterTimeMs = g_now_ms;
    rtpBizPackets.emplace_back(std::move(rtpBizPacket));

    fu_header &= 0x1F;  // clear flags, next loop use
    pkt_cnt++;
    nalu += copy_len;
  }

  return 0;
}

int32_t H264Packetizer::Packetize(
    const std::vector<char>& stream, uint32_t timestamp,
    const std::vector<std::shared_ptr<Extension>>& extensions,
    std::vector<RtpBizPacket>& rtpBizPackets) {
  int ret = 0;

  std::vector<Nalu> nalus;
  if (ParseFrameNalus(stream, nalus) != 0) {
    return -1;
  }

  for (std::vector<Nalu>::iterator iter = nalus.begin(); iter != nalus.end();
       ++iter) {
    if (iter->size <= kGuessMtuByte) {
      if ((ret = PacketSingleNalu(iter->data, iter->size, timestamp, extensions,
                                  rtpBizPackets)) != 0) {
        return -2;
      }
    } else {
      if ((ret = PacketFuA(iter->data, iter->size, timestamp, extensions,
                           rtpBizPackets)) != 0) {
        return -3;
      }
    }
  }

  if (!rtpBizPackets.empty()) {
    reinterpret_cast<RtpHeader*>(rtpBizPackets.back().rtpRawPacket.data())
        ->setMarker(1);
  }

  return 0;
}