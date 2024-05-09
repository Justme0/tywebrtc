#include "src/sdp/sdp_handler.h"

#include "src/pc/peer_connection.h"

namespace tywebrtc {

SdpHandler::SdpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {
  (void)belongingPeerConnection_;
}

}  // namespace tywebrtc
