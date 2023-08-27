#ifndef PULL_PULL_HANDLER_H_
#define PULL_PULL_HANDLER_H_

#include <functional>
#include <string>
#include <vector>

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
                      std::function<int()> handlePacket,
                      std::function<int()> closeFunc);

  // no use
  bool InitSucc() const { return initRet_ == 0; };

  // maybe should input packet blob
  int HandlePacket();

  // tmp public
  const int *p_playSocket_ = nullptr;

 private:
  PeerConnection &belongingPeerConnection_;
  int initRet_ = -1;  // default error

  std::function<int()> initFunc_;
  std::function<int()> handlePacketFunc_;
  std::function<int()> closeFunc_;
};

#endif  // PULL_PULL_HANDLER_H_
