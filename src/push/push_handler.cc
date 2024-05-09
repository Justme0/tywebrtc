#include "src/push/push_handler.h"

#include "src/log/log.h"

namespace tywebrtc {

// @brief setup connection and set av functor
// or set functor in constructor?
int PushHandler::InitPushHandler(
    std::function<int(void)> initFunc, std::function<bool(void)> initSuccFunc,
    std::function<int(const std::vector<char> &frame, uint64_t frameMs)>
        sendAudioFrame,
    std::function<int(const std::vector<char> &frame, uint64_t frameMs)>
        sendVideoFrame) {
  initFunc_ = initFunc;
  initSuccFunc_ = initSuccFunc;
  sendAudioFrameFunc_ = sendAudioFrame;
  sendVideoFrameFunc_ = sendVideoFrame;

  initFunc_();

  return 0;
}

int PushHandler::SendAudioFrame(const std::vector<char> &audioFrame,
                                uint64_t frameMs) {
  int ret = sendAudioFrameFunc_(audioFrame, frameMs);
  if (ret) {
    tylog("sendAudioFrameFunc func ret=%d", ret);
    return ret;
  }

  return 0;
}

int PushHandler::SendVideoFrame(const std::vector<char> &h264Frame,
                                uint64_t frameMs) {
  int ret = sendVideoFrameFunc_(h264Frame, frameMs);
  if (ret) {
    tylog("sendVideoFrameFunc func ret=%d", ret);
    return ret;
  }

  return 0;
}

}  // namespace tywebrtc
