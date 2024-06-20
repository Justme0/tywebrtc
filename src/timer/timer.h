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

class DtlsHandler;
class RtpSender;
class RtpReceiver;
class RtpHandler;

class MonitorStateTimer : public Timer {
 public:
  MonitorStateTimer() : Timer(1000, -1) {}
  ~MonitorStateTimer() override { TimerManager::Instance()->KillTimer(this); }

 private:
  bool _OnTimer() override;
};

class PLITimer : public Timer {
 public:
  PLITimer(RtpReceiver& rtpReceiver)
      : Timer(5000, -1), belongingRtpReceiver_(rtpReceiver) {}

  ~PLITimer() override { TimerManager::Instance()->KillTimer(this); }

 private:
  bool _OnTimer() override;

  RtpReceiver& belongingRtpReceiver_;
};

// DTLS handshake timeout resend
class DTLSTimer : public Timer {
 public:
  DTLSTimer(DtlsHandler& belongingDtlsHandler)
      : Timer(10, -1), belongingDtlsHandler_(belongingDtlsHandler) {}

  ~DTLSTimer() override { TimerManager::Instance()->KillTimer(this); }

 private:
  bool _OnTimer() override;

  DtlsHandler& belongingDtlsHandler_;
};

struct SenderReportTimer : public Timer {
 public:
  SenderReportTimer(RtpSender& rtpSender)
      : Timer(500, -1), belongingRtpSender_(rtpSender) {}
  ~SenderReportTimer() override { TimerManager::Instance()->KillTimer(this); }

 private:
  bool _OnTimer() override;

  RtpSender& belongingRtpSender_;
};

struct ReceiverReportTimer : public Timer {
 public:
  ReceiverReportTimer(RtpReceiver& rtpReceiver)
      : Timer(500, -1), belongingRtpReceiver_(rtpReceiver) {}
  ~ReceiverReportTimer() override { TimerManager::Instance()->KillTimer(this); }

 private:
  bool _OnTimer() override;

  RtpReceiver& belongingRtpReceiver_;
};

struct RembTimer : public Timer {
 public:
  RembTimer(RtpHandler& rtpHandler)
      : Timer(500, -1), belongingRtpHandler_(rtpHandler) {}
  ~RembTimer() override { TimerManager::Instance()->KillTimer(this); }

 private:
  bool _OnTimer() override;

  RtpHandler& belongingRtpHandler_;
};

}  // namespace tywebrtc

#endif  // SRC_TIMER_TIMER_H_