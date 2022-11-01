#ifndef TRANSPORT_RECEIVER_RECEIVER_H_
#define TRANSPORT_RECEIVER_RECEIVER_H_

#include <map>
#include <vector>

#include "rtp/rtp_parser.h"

class RtpReceiver {
 public:
  void PushPacket(std::vector<char>&& vBufReceive);
  std::vector<std::vector<char>> PopOrderedPackets();
  int GetJitterSize() const;

 private:
  // must be ordered, cannot be hashmap
  std::map<PowerSeq, std::vector<char>> jitterBuffer_;

  // mark last popped one, should reserve rtp for nack,
  PowerSeq lastSeq_ = -1;
};

#endif  //   TRANSPORT_RECEIVER_RECEIVER_H_