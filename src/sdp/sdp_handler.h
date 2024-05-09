#ifndef SDP_SDP_HANDLER_H_
#define SDP_SDP_HANDLER_H_

namespace tywebrtc {

class PeerConnection;

class SdpHandler {
 public:
  explicit SdpHandler(PeerConnection &pc);

 public:
  // should be private,
  // chrome default value
  int vp8PayloadType = 96;

 private:
  PeerConnection &belongingPeerConnection_;
};

}  // namespace tywebrtc

#endif  // SDP_SDP_HANDLER_H_
