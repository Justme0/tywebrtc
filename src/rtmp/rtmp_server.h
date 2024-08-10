// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTMP_RTMPSRV_H_
#define SRC_RTMP_RTMPSRV_H_

#include <cstring>
#include <map>

#include "tylib/ip/ip.h"
#include "tylib/string/format_string.h"

#include "src/rtmp/rtmp_assist.h"

namespace tywebrtc {

struct OperateSessionInfo {
  double txn = 0;
  int sid = 0;
  int chan = 0;

  std::string ToString() const {
    return tylib::format_string("{txn=%f, sid=%d, chan=%d}", txn, sid, chan);
  }

  void Reset() {
    txn = 0;
    sid = 0;
    chan = 0;
  }
};

class RtmpServer : public RtmpAssist {
 public:
  int SetupClient(int fd);

  PeerConnection* GetRtmpPeerPC() override;
};

extern char* RtmpPrintTime(unsigned long long uiTime);
extern char* RtmpPrintData(unsigned long long uiTime);
extern in_addr_t GetAddrByName(const char* sIf);

extern unsigned long long GetTimestampMs();

}  // namespace tywebrtc

#endif  // SRC_RTMP_RTMPSRV_H_
