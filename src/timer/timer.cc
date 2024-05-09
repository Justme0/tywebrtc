// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/timer/timer.h"

#include "src/global_tmp/global_tmp.h"
#include "src/monitor/monitor.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

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
  int ret = this->belongingPC_.rtcpHandler_.pli_.CreatePLISend(kSelfRtcpSSRC,
                                                               kMediaSrcSSRC);
  if (ret) {
    tylog("createPLIReportSend ret=%d", ret);
  }

  return true;
}

bool DTLSTimer::_OnTimer() {
  int ret = belongingPC_.dtlsHandler_.OnTime();
  if (ret) {
    tylog("dtls timer ret=%d", ret);
  }

  return true;
}

bool SenderReportTimer::_OnTimer() {
  int ret = belongingPC_.rtcpHandler_.senderReport_.CreateSenderReport();
  if (ret) {
    tylog("timer SR ret=%d.", ret);
  }
  return true;
}

bool ReceiverReportTimer::_OnTimer() {
  int ret = belongingPC_.rtcpHandler_.receiverReport_.CreateReceiverReport();
  if (ret) {
    tylog("timer rr ret=%d.", ret);
  }

  return true;
}

}  // namespace tywebrtc