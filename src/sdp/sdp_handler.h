// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_SDP_SDP_HANDLER_H_
#define SRC_SDP_SDP_HANDLER_H_

namespace tywebrtc {

class PeerConnection;

class SdpHandler {
 public:
  explicit SdpHandler(PeerConnection &pc);

 public:
  // TODO: from SDP
  // should be private,
  // chrome default value
  int vp8PayloadType = 96;
  bool bNotUseSrtp = true;
  bool bUseRsfec = false;
  bool bDtlsSetupActive = false;

 private:
  PeerConnection &belongingPC_;
};

}  // namespace tywebrtc

#endif  // SRC_SDP_SDP_HANDLER_H_
