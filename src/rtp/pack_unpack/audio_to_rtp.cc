#include "rtp/pack_unpack/audio_to_rtp.h"

#include <cassert>

#include "log/log.h"
#include "rtp/pack_unpack/pack_unpack_common.h"
#include "rtp/rtp_parser.h"

int AudioPacketizer::Packetize(const std::vector<char>& stream,
                               uint32_t timestamp,
                               const std::vector<std::shared_ptr<Extension>>&,
                               std::vector<std::vector<char>>& rtp_packets) {
  if (stream.size() + kRtpHeaderLenByte > kGuessMtuByte) {
    return -1;
  }

  std::vector<char> packet(MAX_PKT_BUF_SIZE);
  RtpHeader& header = *reinterpret_cast<RtpHeader*>(packet.data());
  header.setVersion(2);  // fix number
  header.setTimestamp(timestamp);
  header.setSSRC(ssrc_);
  header.setSeqNumber(GetSequence());
  header.setPayloadType(payload_type_);
  header.setMarker(0);
  header.setExtension(0);  // taylor

  const int headLen = header.getHeaderLength();
  char* ptr = packet.data() + headLen;
  memcpy(ptr, stream.data(), stream.size());

  packet.resize(headLen + stream.size());

  rtp_packets.emplace_back(std::move(packet));

  return 0;
}
