// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/sdp/sdp_handler.h"

#include "src/pc/peer_connection.h"

namespace tywebrtc {

SdpHandler::SdpHandler(PeerConnection &pc) : belongingPC_(pc) {
  (void)belongingPC_;
}

}  // namespace tywebrtc
