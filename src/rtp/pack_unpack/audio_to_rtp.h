#ifndef RTP_PACK_UNPACK_AUDIO_TO_RTP_H_
#define RTP_PACK_UNPACK_AUDIO_TO_RTP_H_

#include <cstdint>
#include <vector>

#include "rtp/rtp_parser.h"

class AudioPacketizer {
 public:
  int Packetize(const std::vector<char> &stream, uint32_t timestamp,
                const std::vector<std::shared_ptr<Extension>> &extensions,
                std::vector<RtpBizPacket> &rtpBizPackets);

 private:
  PowerSeqT GeneratePowerSequence() { return ++powerSequence_; }

 private:
  uint8_t payload_type_ = kDownlinkAudioPayloadType;
  uint32_t ssrc_ = kDownlinkAudioSsrc;
  PowerSeqT powerSequence_ = 0;  // last packet seq
  // uint32_t last_timestamp_ = 0;  // last packet timestamp
  // uint32_t max_packet_size_ = 1200;  // mtu
  // RtpPacket::ExtensionTypeIdMap ext_typ_map_;// rtp ext
};

#endif  // RTP_PACK_UNPACK_AUDIO_TO_RTP_H_
