#include "transport/sender/sender.h"

#include <cassert>

#include "rtp/rtp_handler.h"

RtpSender::RtpSender(SSRCInfo& ssrcInfo)
    : belongingSSRCInfo_(ssrcInfo), saveLast_(sendQueue_.end()) {}

// push to sender queue, and update SSRCInfo biggest cycle+seq
void RtpSender::Enqueue(RtpBizPacket&& rtpBizPacket) {
  tylog("enqueue biz pkt=%s.", rtpBizPacket.ToString().data());

  sendQueue_.emplace(rtpBizPacket.GetPowerSeq(), std::move(rtpBizPacket));
  assert(rtpBizPacket.rtpRawPacket.empty());

  PowerSeqT biggestPowerSeq = (--sendQueue_.end())->first;

  // https://stackoverflow.com/questions/22520151/how-to-treat-stdpair-as-two-separate-variables
  std::tie(belongingSSRCInfo_.biggestCycle, belongingSSRCInfo_.biggestSeq) =
      SplitPowerSeq(biggestPowerSeq);

  if (sendQueue_.size() > kSendQueueSaveLen) {
    sendQueue_.erase(sendQueue_.begin());
  }

  tylog("after enqueue, queue size=%zu.", sendQueue_.size());
}

// To avoid copy, return value points to internal memory, note concurrent
// problem, output only modify pointer.
// OPT: use optional to indicate NULL
const std::vector<char>* RtpSender::GetSeqPacket(PowerSeqT powerSeq) const {
  auto it = sendQueue_.find(powerSeq);
  if (it == sendQueue_.end()) {
    std::stringstream ss;
    for (const auto& elem : sendQueue_) {
      ss << " " << elem.first;
    }

    tylog("not found powerSeq=%ld, queue size=%zu: %s.", powerSeq,
          sendQueue_.size(), ss.str().data());

    return nullptr;
  }

  return &it->second.rtpRawPacket;
}

// int RtpSender::GetQueueSize() const { return this->sendQueue_.size(); }

// 加入queue时已保证有序和完整
// to pacing
// no use now
std::vector<RtpBizPacket> RtpSender::Dequeue() {
  std::vector<RtpBizPacket> packets;

  std::map<PowerSeqT, RtpBizPacket>::const_iterator firstPopIt;
  if (saveLast_ == sendQueue_.end()) {
    firstPopIt = sendQueue_.begin();
  } else {
    firstPopIt = std::next(saveLast_);
  }

  for (auto it = firstPopIt; it != sendQueue_.end(); ++it) {
    // packets.push_back(it->second);
    saveLast_ = it;

    if (sendQueue_.size() > kSendQueueSaveLen) {
      sendQueue_.erase(sendQueue_.begin());
    }
  }
  tylog("after dequeue, size=%zu.", sendQueue_.size());

  return packets;
}