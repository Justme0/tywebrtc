#include "rtp/rtcp/rtcp_nack.h"

#include "rtp/rtcp/rtcp_parser.h"

static int SerializeNack(const std::vector<NackBlock>& nackBlokVect,
                         uint32_t sinkSSRC, uint32_t soucreSSRC,
                         std::vector<char>& o_rtcpBin) {
  RtcpHeader nack;
  nack.setPacketType(RtcpPacketType::kGenericRtpFeedback);
  nack.setBlockCount(1);
  nack.setSSRC(sinkSSRC);
  nack.setSourceSSRC(soucreSSRC);
  nack.setLength(2 + nackBlokVect.size());

  if (12 + nackBlokVect.size() * 4 > 2048) {
    tylog("nack pkg len > 2048");

    return -1;
  }

  const char* head = reinterpret_cast<const char*>(&nack);
  o_rtcpBin.assign(head, head + 12);

  const char* body = reinterpret_cast<const char*>(&nackBlokVect[0]);
  o_rtcpBin.insert(o_rtcpBin.end(), body, body + nackBlokVect.size() * 4);

  return 0;
}

int CreateNackReport(const std::set<int>& lostSeqs, uint32_t localSSRC,
                     uint32_t remoteSSRC, std::vector<char>& o_rtcpPacketBin) {
  std::vector<NackBlock> nackBlokVect;
  for (auto it = lostSeqs.begin(); it != lostSeqs.end();) {
    uint16_t pid = *it;
    uint16_t blp = 0;

    for (++it; it != lostSeqs.end(); ++it) {
      uint16_t diff = *it - pid - 1;

      if (diff >= 16) {
        break;
      }

      blp |= (1 << diff);
    }

    NackBlock block;
    block.setNackPid(pid);
    block.setNackBlp(blp);
    nackBlokVect.push_back(block);
  }

  return SerializeNack(nackBlokVect, localSSRC, remoteSSRC, o_rtcpPacketBin);
}