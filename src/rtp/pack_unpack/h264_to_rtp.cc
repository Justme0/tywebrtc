#include "rtp/pack_unpack/h264_to_rtp.h"

#include "codec_utility.h"
#include "common/log.h"
#include "common/system_adapt.h"
#include "common/tools.h"
#include "peerconnection/pc_data.h"
#include "rtp/rtp.h"

// H.264 nalu header type mask.
const uint8_t kNalTypeMask = 0x1F;

// @see: https://tools.ietf.org/html/rfc6184#section-5.8

struct Nalu {
  const uint8_t* data;
  int size;
};

static int ParseFrameNalus(const uint8_t* frame, uint32_t len,
                           std::vector<Nalu>& nalus) {
  if (len < 4) {
    return -1;
  }

  if (frame[0] == 0x00 && frame[1] == 0x00 && frame[2] == 0x00 &&
      frame[3] == 0x01) {
    // Annex-B
    // 00 00 00 01 ab cd ef 00 00 00 01
    int pos = 0;
    const uint8_t* nal = NULL;
    while (pos + 3 < len) {
      if (frame[pos] != 0x00 || frame[pos + 1] != 0x00 ||
          frame[pos + 2] != 0x01) {
        ++pos;
        continue;
      }

      if (nal != NULL) {
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

    if (nal != NULL) {
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
    uint32_t pos = 0;
    while (pos < len) {
      uint32_t nal_len = (frame[pos + 0] << 24) | (frame[pos + 1] << 16) |
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

H264Packetizer::H264Packetizer(uint32_t ssrc, uint32_t mtu, uint32_t sequence,
                               uint8_t payload_typ,
                               const std::string& stream_id)
    : Packetizer(ssrc, mtu, sequence, payload_typ, stream_id) {}

H264Packetizer::~H264Packetizer() {}

int32_t H264Packetizer::Packetize(
    const uint8_t* data, uint32_t length, uint32_t timestamp,
    const RtpHeader::ExtensionList& extensions,
    std::vector<std::unique_ptr<RtpPacket>>& rtp_packets) {
  int ret = VIDEO_RTP_OK;

  WEBRTC_ERROR_CHECK(stream_id_.c_str(), (VIDEO_NAL_START_CODE_LEN >= length),
                     -2, "data buf is NULL!");
  WEBRTC_ERROR_CHECK(stream_id_.c_str(), (NULL == data), 0,
                     "left data len error: %d. return.", length);

  std::vector<Nalu> nalus;
  if (ParseFrameNalus(data, length, nalus) != 0) {
    return -1;
  }

  for (std::vector<Nalu>::iterator iter = nalus.begin(); iter != nalus.end();
       ++iter) {
    if (iter->size <= max_packet_size_) {
      if ((ret = PacketSingleNalu(iter->data, iter->size, timestamp, extensions,
                                  rtp_packets)) != 0) {
        return -1;
      }
    } else {
      if ((ret = PacketFuA(iter->data, iter->size, timestamp, extensions,
                           rtp_packets)) != 0) {
        return -1;
      }
    }
  }

  if (!rtp_packets.empty()) {
    const auto& last_pkt = rtp_packets.back();
    RtpPacketImpl* pkt = static_cast<RtpPacketImpl*>(last_pkt.get());
    pkt->header.marker = true;
    pkt->data->data()[1] =
        (uint8_t)((pkt->header.marker << 7) | pkt->header.payload_type);
  }

  return ret;
}

int H264Packetizer::PacketFuA(
    const uint8_t* frame, const int len, const uint32_t timestamp,
    const RtpHeader::ExtensionList& extensions,
    std::vector<std::unique_ptr<RtpPacket>>& packets) {
  WEBRTC_ERROR_CHECK(stream_id_.c_str(), (nullptr == frame), 0,
                     "Chn %" PRIu64 " rtp buf is NULL!", ssrc_);
  WEBRTC_ERROR_CHECK(stream_id_.c_str(), (VIDEO_NALU_HEADER_LTH >= len), 0,
                     "Chn %" PRIu64 " rtp data len %d error.", ssrc_, len);

  if ((frame[0] & kH264TypeMask) == 0x05) {
    PacketStapA(timestamp, extensions, packets);
  }

  int32_t data_len = len - VIDEO_NALU_HEADER_LTH;
  int32_t slice_len = max_packet_size_ - RTP_HEADER_LTH;
  assert(slice_len > 0);
  int32_t pkt_num = data_len / slice_len;

  if (0 >= pkt_num) {
    pkt_num = 0;
    slice_len = data_len;
  }

  int32_t last_pkt_len = data_len - pkt_num * slice_len;
  pkt_num += ((0 < last_pkt_len) ? 1 : 0);

  if (0 < pkt_num) {
    slice_len = (data_len + (pkt_num >> 1)) / pkt_num;
  } else {
    pkt_num = 1;
    slice_len = data_len;
  }

  const uint8_t* nalu = frame + VIDEO_NALU_HEADER_LTH;
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

    std::unique_ptr<RtpPacket> packet = RtpPacket::Create(MAX_PKT_BUF_SIZE);
    RtpPacketImpl* pkt = static_cast<RtpPacketImpl*>(packet.get());
    RtpHeader& header = pkt->header;
    header.version = 2;
    header.timestamp = timestamp;
    header.ssrc = ssrc_;
    header.sequence_number = GetSequence();
    header.payload_type = payload_type_;
    header.marker = 0;
    header.extensions = extensions;
    pkt->header.header_length = Rtp::CreateHeader(
        pkt->data->data(), MAX_PKT_BUF_SIZE, header, ext_typ_map_);
    pkt->data->length = pkt->header.header_length + VIDEO_FU_INDICATOR_SIZE +
                        VIDEO_FU_HEADER_SIZE + copy_len;

    uint8_t* cur_pos = pkt->data->data() + pkt->header.header_length;
    if ((pkt_num - 1) == i) {
      fu_header = kEBit | (fu_header & 0x1F);  // FU-A end
    }
    *cur_pos++ = fu_indicator;
    *cur_pos++ = fu_header;

    memcpy(cur_pos, nalu, copy_len);

    packets.emplace_back(std::move(packet));

    fu_header &= 0x1F;  // clear flags
    pkt_cnt++;
    nalu += copy_len;
  }

  return 0;
}

int H264Packetizer::PacketSingleNalu(
    const uint8_t* frame, const int len, const uint32_t timestamp,
    const RtpHeader::ExtensionList& extensions,
    std::vector<std::unique_ptr<RtpPacket>>& packets) {
  if ((frame[0] & kNalTypeMask) == 0x07) {
    sps_.assign(reinterpret_cast<const char*>(frame), len);
    pps_.clear();
    // return 0;
  }

  if ((frame[0] & kNalTypeMask) == 0x08) {
    pps_.append(reinterpret_cast<const char*>(frame), len);
    // pps_.assign(reinterpret_cast<const char*>(frame), len);
    // return 0;
  }

  if ((frame[0] & kNalTypeMask) == 0x05) {
    PacketStapA(timestamp, extensions, packets);
  }

  int ret = 0;

  std::unique_ptr<RtpPacket> packet = RtpPacket::Create(MAX_PKT_BUF_SIZE);
  RtpPacketImpl* pkt = static_cast<RtpPacketImpl*>(packet.get());
  RtpHeader& header = pkt->header;
  header.version = 2;
  header.timestamp = timestamp;
  header.ssrc = ssrc_;
  header.sequence_number = GetSequence();
  header.payload_type = payload_type_;
  header.marker = 0;
  header.extensions = extensions;
  int32_t header_len = Rtp::CreateHeader(pkt->data->data(), MAX_PKT_BUF_SIZE,
                                         header, ext_typ_map_);
  header.header_length = header_len;
  auto ptr = pkt->data->data() + header_len;
  memcpy(ptr, reinterpret_cast<const char*>(frame), len);
  pkt->data->length = header.header_length + len;

  packets.emplace_back(std::move(packet));

  return ret;
}

int H264Packetizer::PacketStapA(
    const uint32_t timestamp, const RtpHeader::ExtensionList& extensions,
    std::vector<std::unique_ptr<RtpPacket>>& packets) {
  if (sps_.empty() || pps_.empty()) {
    return 0;
  }

  int ret = 0;

  uint8_t header = sps_[0];
  uint8_t nal_type = header & kNalTypeMask;

  std::unique_ptr<RtpPacket> packet = RtpPacket::Create(MAX_PKT_BUF_SIZE);
  RtpPacketImpl* pkt = static_cast<RtpPacketImpl*>(packet.get());
  auto dst = pkt->data->data();
  RtpHeader& rtp_header = pkt->header;
  rtp_header.version = 2;
  rtp_header.timestamp = timestamp;
  rtp_header.ssrc = ssrc_;
  rtp_header.sequence_number = GetSequence();
  rtp_header.payload_type = payload_type_;
  rtp_header.marker = 0;
  rtp_header.extensions = extensions;
  int32_t header_len =
      Rtp::CreateHeader(dst, MAX_PKT_BUF_SIZE, rtp_header, ext_typ_map_);
  rtp_header.header_length = header_len;
  dst += header_len;

  // stap-a header
  uint8_t stap_a_header = kH264StapA;
  stap_a_header |= (nal_type & (~kNalTypeMask));
  *dst++ = stap_a_header;

  WriteBigEndian(dst, sps_.size(), 2);
  dst += 2;
  memcpy(dst, (const uint8_t*)sps_.data(), sps_.size());
  dst += sps_.size();
  WriteBigEndian(dst, pps_.size(), 2);
  dst += 2;
  memcpy(dst, (const uint8_t*)pps_.data(), pps_.size());
  dst += pps_.size();

  pkt->data->length = dst - pkt->data->data();

  packets.emplace_back(std::move(packet));

  return ret;
}
