// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

// all timer task

#ifndef SRC_TIMER_TIMER_H_
#define SRC_TIMER_TIMER_H_

#include "tylib/time/timer.h"

namespace tywebrtc {

class PeerConnection;

class MonitorStateTimer : public Timer {
 public:
  MonitorStateTimer() : Timer(1000, -1) {}

 private:
  bool _OnTimer() override;
};

// DTLS handshake timeout resend
class DTLSTimer : public Timer {
 public:
  DTLSTimer(PeerConnection& belongingPC)
      : Timer(10, -1), belongingPC_(belongingPC) {}

 private:
  bool _OnTimer() override;

  PeerConnection& belongingPC_;
};

// RTCP
class PLITimer : public Timer {
 public:
  PLITimer(PeerConnection& belongingPC)
      : Timer(4000, -1), belongingPC_(belongingPC) {}

 private:
  bool _OnTimer() override;

  PeerConnection& belongingPC_;
};

struct SenderReportTimer : public Timer {
 public:
  SenderReportTimer(PeerConnection& belongingPC)
      : Timer(500, -1), belongingPC_(belongingPC) {}

 private:
  bool _OnTimer() override;

  PeerConnection& belongingPC_;
};

struct ReceiverReportTimer : public Timer {
 public:
  ReceiverReportTimer(PeerConnection& belongingPC)
      : Timer(500, -1), belongingPC_(belongingPC) {}

 private:
  bool _OnTimer() override;

  PeerConnection& belongingPC_;
};

}  // namespace tywebrtc

#endif  // SRC_TIMER_TIMER_H_