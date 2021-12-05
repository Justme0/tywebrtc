#ifndef SDP_SDP_HANDLER_H_
#define SDP_SDP_HANDLER_H_

class PeerConnection;

class SdpHandler {
 public:
  explicit SdpHandler(PeerConnection &pc);

 private:
  PeerConnection &belongingPeerConnection_;
};

#endif  // SDP_SDP_HANDLER_H_
