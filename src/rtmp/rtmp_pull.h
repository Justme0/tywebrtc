// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTMP_RTMP_PULL_H_
#define SRC_RTMP_RTMP_PULL_H_

#include "src/rtmp/rtmp_assist.h"

namespace tywebrtc {

class RtmpPuller : public RtmpAssist {
 public:
  explicit RtmpPuller(PeerConnection& pc);

  int InitProtocolHandler(const std::string& Url);

  PeerConnection* GetRtmpPeerPC() override { return &belongingPC_; }

  PeerConnection& belongingPC_;
};

}  // namespace tywebrtc

#endif  // SRC_RTMP_RTMP_PULL_H_
