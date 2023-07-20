// all timer task

#ifndef TIMER_TIMER_H_
#define TIMER_TIMER_H_

#include "tylib/time/timer.h"

class PeerConnection;

class MonitorStateTimer : public Timer {
 public:
  MonitorStateTimer() : Timer(1000, -1) {}

 private:
  bool _OnTimer() override;
};

class PLITimer : public Timer {
 public:
  PLITimer(PeerConnection& belongingPC)
      : Timer(4000, -1), belongingPC_(belongingPC) {}

 private:
  bool _OnTimer() override;

  PeerConnection& belongingPC_;
};

class DTLSTimer : public Timer {
 public:
  DTLSTimer(PeerConnection& belongingPC)
      : Timer(10, -1), belongingPC_(belongingPC) {}

 private:
  bool _OnTimer() override;

  PeerConnection& belongingPC_;
};

#endif  //  TIMER_TIMER_H_