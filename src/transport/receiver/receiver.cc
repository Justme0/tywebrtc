#include "transport/receiver/receiver.h"

void RtpReceiver::PushPacket(std::vector<char>&& vBufReceive)
{
	// fix: power seq is related to ssrc
  const RtpHeader& rtpHeader = *reinterpret_cast<const RtpHeader*>(vBufReceive.data());
  this->jitterBuffer_[rtpHeader.getPowerSeq()] = vBufReceive;
}

void RtpReceiver::GetJitterSize() const{
	return jitterBuffer_.size();
}

 std::vector<std::vector<char>> RtpReceiver::PopOrderedPackets(){
 }