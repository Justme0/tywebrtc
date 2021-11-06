#pragma once

class PeerConnection;

class RTPHandler {
public:
  RTPHandler(PeerConnection &pc);

  PeerConnection &belongingPeerConnection_;
};
