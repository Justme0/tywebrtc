#ifndef SRC_RTP_RTCP_RTCP_PLI_H_
#define SRC_RTP_RTCP_RTCP_PLI_H_

#include <cstdint>

#include "src/rtp/rtcp/rtcp_parser.h"

namespace tywebrtc {

class RtcpHandler;
class RtcpHeader;

class RtcpPLI {
 public:
  explicit RtcpPLI(RtcpHandler &belongingRtcpHandler);
  int HandlePLI(const RtcpHeader &chead);

  int CreatePLISend(uint32_t ssrc, uint32_t sourceSSRC);

 private:
  RtcpHandler &belongingRtcpHandler_;
};

}  // namespace tywebrtc

#endif  // SRC_RTP_RTCP_RTCP_PLI_H_
