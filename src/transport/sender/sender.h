#ifndef TRANSPORT_SENDER_SENDER_H_
#define TRANSPORT_SENDER_SENDER_H_

#include <map>
#include <vector>

#include "rtp/rtp_parser.h"

// audio should not in pacing
class RtpSender {
 public:
  void Enqueue(std::vector<char>&& vBufReceive);
  std::vector<std::vector<char>> Dequeue();
  int GetQueueSize() const;

 private:
  std::map<PowerSeq, std::vector<char>> sendQueue_;
};

#endif  //   TRANSPORT_SENDER_SENDER_H_