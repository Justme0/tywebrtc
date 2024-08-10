// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/pull/pull_handler.h"

#include <cassert>

#include "src/log/log.h"

namespace tywebrtc {

PullHandler::PullHandler(PeerConnection &) {}

// TODO: push handler must destructor
PullHandler::~PullHandler() {
  // taylor FIXME
  if (closeFunc_) {
    assert(closeFunc_);  // callable
    int ret = closeFunc_();
    if (ret) {
      tylog("close ret=%d.", ret);
    }
  }
}

// @brief setup connection and set av functor
// or set functor in constructor?
int PullHandler::InitPullHandler(const int *p_playSocket,
                                 std::function<int()> initFunc,
                                 std::function<int(int)> handlePacketFunc,
                                 std::function<int()> closeFunc) {
  p_playSocket_ = p_playSocket;
  initFunc_ = initFunc;
  handlePacketFunc_ = handlePacketFunc;
  closeFunc_ = closeFunc;

  initRet_ = initFunc_();
  if (initRet_) {
    tylog("init func ret=%d", initRet_);
    return initRet_;
  }

  return 0;
}

// now no use
int PullHandler::HandlePacket() {
  // int ret = handlePacketFunc_();
  // if (ret) {
  //   tylog("handlePacketFunc func ret=%d", ret);
  //   return ret;
  // }

  return 0;
}

}  // namespace tywebrtc
