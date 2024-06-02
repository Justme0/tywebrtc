// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_SIGNAL_SIGNAL_HANDLER_H_
#define SRC_SIGNAL_SIGNAL_HANDLER_H_

namespace tywebrtc {

class PeerConnection;

class SignalHandler {
 public:
  explicit SignalHandler(PeerConnection& pc);
  int S2CReportRTT(int rttMs);

  // tmp
 public:
  PeerConnection& belongingPC_;
};

}  // namespace tywebrtc

#endif  // SRC_SIGNAL_SIGNAL_HANDLER_H_