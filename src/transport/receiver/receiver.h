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
 std::map<PowerSeqT, std::vector<char>> jitterBuffer_;
 
};

#endif  //   TRANSPORT_RECEIVER_RECEIVER_H_