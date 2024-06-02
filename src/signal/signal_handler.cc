// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/signal/signal_handler.h"

#include "google/protobuf/util/json_util.h"
#include "tylib/string/any_to_string.h"

#include "biz_proto/client.pb.h"
#include "src/log/log.h"
#include "src/pc/peer_connection.h"
namespace tywebrtc {

SignalHandler::SignalHandler(PeerConnection& pc) : belongingPC_(pc) {}

int SignalHandler::S2CReportRTT(int rttMs) {
  int ret = 0;

  client::AllProto pb;
  auto state = pb.mutable_s2c_state_req();
  state->set_str_server_time_ms(tylib::AnyToString(g_now_ms));
  state->set_uint32_rtt_ms(rttMs);
  tylog("pb=%s.", pb.Utf8DebugString().data());

  std::string s;
  google::protobuf::util::Status status =
      google::protobuf::util::MessageToJsonString(pb, &s);
  if (!status.ok()) {
    tylog("pb to json err=%s.", status.ToString().data());

    return -1;
  }
  tylog("json=%s.", s.data());

  ret = this->belongingPC_.dataChannelHandler_.SendSctpDataForLable(s);
  if (ret) {
    tylog("send data channel ret=%d.", ret);

    return ret;
  }

  return 0;
}

}  // namespace tywebrtc
