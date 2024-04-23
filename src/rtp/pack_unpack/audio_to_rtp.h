// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTP_PACK_UNPACK_AUDIO_TO_RTP_H_
#define SRC_RTP_PACK_UNPACK_AUDIO_TO_RTP_H_

#include <cstdint>
#include <vector>

#include "src/rtp/rtp_parser.h"

namespace tywebrtc {

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

}  // namespace tywebrtc

#endif  // SRC_RTP_PACK_UNPACK_AUDIO_TO_RTP_H_