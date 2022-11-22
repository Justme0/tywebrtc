#ifndef RTP_RTCP_RTCP_NACK_H_
#define RTP_RTCP_RTCP_NACK_H_

#include <set>
#include <vector>

#include <cstdint>

int CreateNackReport(const std::set<int>& lostSeqs, uint32_t localSSRC,
                     uint32_t remoteSSRC, std::vector<char>& o_rtcpPacketBin);
// #include "rtp/rtcp/rtcp_packet.h"

// class RTCPNACK : public RTCPPacket {
//  public:
//   RTCPNACK();
//   virtual uint32_t GetSize();
//   virtual uint32_t Parse(const uint8_t* data, uint32_t size);
//   virtual uint32_t Serialize(uint8_t* data, uint32_t size);
//
//  private:
//   uint32_t ssrc;
//   uint16_t fsn;
//   uint16_t blp;
// };

#endif  // RTP_RTCP_RTCP_NACK_H_
