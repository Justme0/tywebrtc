#include "transport/sender/sender.h"

#include <cassert>

void RtpSender::Enqueue(std::vector<char>&& vBufReceive) {
  // fix: power seq is related to ssrc
  const RtpHeader& rtpHeader =
      *reinterpret_cast<const RtpHeader*>(vBufReceive.data());
  this->sendQueue_[rtpHeader.getPowerSeq()] = std::move(vBufReceive);
  assert(vBufReceive.empty());
}

int RtpSender::GetQueueSize() const { return this->sendQueue_.size(); }

// 加入queue时已保证有序和完整
// should pacing
std::vector<std::vector<char>> RtpSender::Dequeue() {
  std::vector<std::vector<char>> packets;

  for (auto it = sendQueue_.begin(); it != sendQueue_.end();) {
    tylog("pop from sender queue seq=%ld", it->first);
    packets.emplace_back(std::move(it->second));
    assert(it->second.empty());

    it = sendQueue_.erase(it);
  }

  return packets;
}