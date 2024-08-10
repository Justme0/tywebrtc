// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTMP_RTMP_PUSH_H_
#define SRC_RTMP_RTMP_PUSH_H_

#include <cstdint>
#include <string>
#include <vector>

#include "librtmp/rtmp.h"

#include "src/push/push_handler.h"
#include "src/rtmp/FlvAssist.h"

namespace tywebrtc {

// #include "user_basic_info_struct.h"

const uint32_t DEFAULT_TIMEOUT = 30;  // 30 seconds

class PeerConnection;

class RtmpPusher {
 public:
  explicit RtmpPusher(PeerConnection &pc);

  // note assignment and copy constructor
  ~RtmpPusher();

  int InitProtocolHandler(const std::string &rtmpUrl);
  bool InitSucc() const;

  int SendAudioFrame(const std::vector<char> &audioFrame, uint64_t frameMs);
  int SendVideoFrame(const std::vector<char> &h264Frame, uint64_t frameMs);

  // now no use
  int RecvRtmpPacket(std::vector<char> *o_recvBuf);

 private:
  int InitRtmp_();
  int SetupRtmp_(std::string rtmpUrl, bool writeable, uint32_t timeout,
                 bool liveSource);
  int ConnectRtmp_();

  int ReconnectRtmp_();

 private:
  // PeerConnection &belongingPC_;
  FlvAssist flvAssist;
  // uint32_t mMaxRtmpPacketLength = 0;

  std::string mRtmpUrl;
  bool mWriteable = false;
  uint32_t mTimeOut = 30;  // second
  bool mLiveSource = false;
  bool initSucc_ = false;

 public:
  // tmp public, indicate if rtmp setup is ok (not null)
  RTMP *mRtmpInstance = nullptr;
};

}  // namespace tywebrtc

#endif  // SRC_RTMP_RTMP_PUSH_H_
