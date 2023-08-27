#ifndef PUSH_PUSH_HANDLER_H_
#define PUSH_PUSH_HANDLER_H_

#include <functional>
#include <string>
#include <vector>

class PushHandler {
 public:
  PushHandler() = default;

  // delete function should be public
  // https://stackoverflow.com/questions/55205874/deletion-of-copy-ctor-copy-assignment-public-private-or-protected
  PushHandler(const PushHandler &) = delete;
  PushHandler &operator=(const PushHandler &) = delete;

  int InitPushHandler(
      std::function<int(void)> initFunc, std::function<bool(void)> initSuccFunc,
      std::function<int(const std::vector<char> &frame, uint64_t frameMs)>
          sendAudioFrame,
      std::function<int(const std::vector<char> &frame, uint64_t frameMs)>
          sendVideoFrame);

  // must check if empty, otherwise throw.
  // https://stackoverflow.com/questions/21806632/how-to-properly-check-if-stdfunction-is-empty-in-c11
  bool InitSucc() const { return initSuccFunc_ && initSuccFunc_(); };

  int SendAudioFrame(const std::vector<char> &audioFrame, uint64_t frameMs);
  int SendVideoFrame(const std::vector<char> &h264Frame, uint64_t frameMs);

 private:
  std::function<int(void)> initFunc_;
  std::function<bool(void)> initSuccFunc_;
  std::function<int(const std::vector<char> &frame, uint64_t frameMs)>
      sendAudioFrameFunc_;
  std::function<int(const std::vector<char> &frame, uint64_t frameMs)>
      sendVideoFrameFunc_;
};

#endif  // PUSH_PUSH_HANDLER_H_
