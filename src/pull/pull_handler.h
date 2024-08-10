// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_PULL_PULL_HANDLER_H_
#define SRC_PULL_PULL_HANDLER_H_

#include <functional>
#include <string>
#include <vector>

namespace tywebrtc {

class PeerConnection;

class PullHandler {
 public:
  int shitFd = 0;

  explicit PullHandler(PeerConnection &pc);
  ~PullHandler();

  // delete function should be public
  // https://stackoverflow.com/questions/55205874/deletion-of-copy-ctor-copy-assignment-public-private-or-protected
  PullHandler(const PullHandler &) = delete;
  PullHandler &operator=(const PullHandler &) = delete;

  int InitPullHandler(const int *p_playSocket, std::function<int()> initFunc,
                      std::function<int(int)> handlePacket,
                      std::function<int()> closeFunc);

  // no use
  bool InitSucc() const { return initRet_ == 0; };

  // maybe should input packet blob
  int HandlePacket();

  // tmp public
  const int *p_playSocket_ = nullptr;

 private:
  // PeerConnection &belongingPC_;
  int initRet_ = -1;  // default error

  std::function<int()> initFunc_;
  std::function<int(int)> handlePacketFunc_;
  std::function<int()> closeFunc_;
};

}  // namespace tywebrtc

#endif  // SRC_PULL_PULL_HANDLER_H_
