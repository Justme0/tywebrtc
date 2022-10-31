#include "transport/receiver/receiver.h"

#include <cassert>

void RtpReceiver::PushPacket(std::vector<char>&& vBufReceive) {
  // if recv same packet, no problem.
  // fix: power seq is related to ssrc
  const RtpHeader& rtpHeader =
      *reinterpret_cast<const RtpHeader*>(vBufReceive.data());
  this->jitterBuffer_[rtpHeader.getPowerSeq()] = std::move(vBufReceive);
  assert(vBufReceive.empty());
}

int RtpReceiver::GetJitterSize() const { return jitterBuffer_.size(); }

// OPT: not wait stategy
std::vector<std::vector<char>> RtpReceiver::PopOrderedPackets() {
  std::vector<std::vector<char>> orderedPackets;

  for (auto it = jitterBuffer_.begin();
       it != jitterBuffer_.end() &&
       (-1 == lastSeq_ || it->first == lastSeq_ + 1);) {
    tylog("add to jitter seq=%ld", it->first);
    orderedPackets.emplace_back(std::move(it->second));
    lastSeq_ = it->first;

    it = jitterBuffer_.erase(it);
  }

  return orderedPackets;
}