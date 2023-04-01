#include "timer/timer.h"

#include "global_tmp/global_tmp.h"
#include "monitor/monitor.h"
#include "pc/peer_connection.h"

bool MonitorStateTimer::_OnTimer() {
  Singleton<PCManager>::Instance().CleanTimeoutPeerConnection();

  const int size = Singleton<PCManager>::Instance().GetPeerConnectionSize();
  tylog("client2pc size=%d.", size);

  static prometheus::Family<prometheus::Gauge>* s_sessionNum = nullptr;
  // first time maybe null
  if (nullptr != s_sessionNum) {
    // https://stackoverflow.com/questions/45172765/how-to-remove-no-longer-valid-gauges
    g_pRegistry->Remove(*s_sessionNum);
  }
  s_sessionNum = &prometheus::BuildGauge()
                      .Name("session_pcu")
                      .Help("Number of sessions")
                      .Register(*g_pRegistry);

  for (auto& session : Singleton<PCManager>::Instance().client2PC_) {
    s_sessionNum
        ->Add({{"clientIP", session.first.ip},
               {"clientPort", tylib::AnyToString(session.first.port)}})
        .Set(1);
  }

  return true;
}

bool PLITimer::_OnTimer() {
  const uint32_t kSelfRtcpSSRC = 1;
  const uint32_t kMediaSrcSSRC = this->belongingPC_.rtpHandler_.upVideoSSRC;
  int ret = this->belongingPC_.rtcpHandler_.CreatePLIReportSend(kSelfRtcpSSRC,
                                                                kMediaSrcSSRC);
  if (ret) {
    tylog("createPLIReportSend ret=%d", ret);
  }

  return true;
}

bool DTLSTimer::_OnTimer() {
  int ret = belongingPC_.dtlsHandler_.OnTime();
  if (ret) {
    tylog("createPLIReportSend ret=%d", ret);
  }

  return true;
}