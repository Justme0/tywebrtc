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
  tylog("on timer");
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
  tylog("on timer");
  int ret = belongingRtpReceiver_.belongingSSRCInfo_.belongingRtpHandler
                .belongingPC_.rtcpHandler_.psfb_.pli_.CreatePLISend();
  if (ret) {
    tylog("create pli ret=%d", ret);
    return true;
  }

  return true;
}

bool DTLSTimer::_OnTimer() {
  tylog("on timer");
  int ret = belongingDtlsHandler_.OnTime();
  if (ret) {
    tylog("dtls timer ret=%d", ret);
    return true;
  }

  return true;
}

bool SenderReportTimer::_OnTimer() {
  tylog("on timer");

  std::vector<char> rtcpBin;

  int ret = belongingRtpSender_.belongingSSRCInfo_.belongingRtpHandler
                .belongingPC_.rtcpHandler_.senderReport_.CreateSenderReport(
                    belongingRtpSender_, &rtcpBin);
  if (ret) {
    tylog("create sr ret=%d.", ret);

    return true;
  }

  ret = belongingRtpSender_.belongingSSRCInfo_.belongingRtpHandler.belongingPC_
            .rtcpHandler_.extendedReport_.dlrr_.CreateRtcpDLRR(
                belongingRtpSender_, &rtcpBin);
  if (ret) {
    tylog("create dlrr ret=%d.", ret);

    return true;
  }

  if (rtcpBin.empty()) {
    tylog("rtcp bin is empty, have no send");
    return true;
  }

  DumpSendPacket(rtcpBin);
  ret = this->belongingRtpSender_.belongingSSRCInfo_.belongingRtpHandler
            .belongingPC_.srtpHandler_.ProtectRtcp(
                const_cast<std::vector<char>*>(&rtcpBin));
  if (ret) {
    tylog("send to client, protect rtcp ret=%d", ret);

    return true;
  }
  ret = this->belongingRtpSender_.belongingSSRCInfo_.belongingRtpHandler
            .belongingPC_.SendToClient(rtcpBin);
  if (ret) {
    tylog("send to client ret=%d", ret);

    return true;
  }

  return true;
}

bool ReceiverReportTimer::_OnTimer() {
  tylog("on timer");
  std::vector<char> rtcpBin;
  int ret = belongingRtpReceiver_.belongingSSRCInfo_.belongingRtpHandler
                .belongingPC_.rtcpHandler_.receiverReport_.CreateReceiverReport(
                    belongingRtpReceiver_, &rtcpBin);
  if (ret) {
    tylog("create rr ret=%d.", ret);

    return true;
  }

  ret = belongingRtpReceiver_.belongingSSRCInfo_.belongingRtpHandler
            .belongingPC_.rtcpHandler_.extendedReport_.rrtr_.CreateRtcpRRTR(
                belongingRtpReceiver_.belongingSSRCInfo_.ssrc_key_, &rtcpBin);
  if (ret) {
    tylog("create rrtr ret=%d.", ret);

    return true;
  }

  DumpSendPacket(rtcpBin);
  ret = this->belongingRtpReceiver_.belongingSSRCInfo_.belongingRtpHandler
            .belongingPC_.srtpHandler_.ProtectRtcp(
                const_cast<std::vector<char>*>(&rtcpBin));
  if (ret) {
    tylog("send to client, protect rtcp ret=%d", ret);

    return true;
  }
  ret = this->belongingRtpReceiver_.belongingSSRCInfo_.belongingRtpHandler
            .belongingPC_.SendToClient(rtcpBin);
  if (ret) {
    tylog("send to client ret=%d", ret);

    return true;
  }

  return true;
}

}  // namespace tywebrtc