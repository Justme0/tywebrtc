#include "sdp/sdp_handler.h"

#include "pc/peer_connection.h"

SdpHandler::SdpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {
  (void)belongingPeerConnection_;
}
