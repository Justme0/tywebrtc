// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtp/rtcp/rtcp_packet/rtp_fb/rtcp_nack.h"

#include <cassert>

#include "src/pc/peer_connection.h"
#include "src/rtp/rtcp/rtcp_parser.h"

namespace tywebrtc {

RtcpNack::RtcpNack(RtcpRtpFeedback &belongingRtpfb)
    : belongingRtpfb_(belongingRtpfb) {}

// downlink send lost package
int RtcpNack::SendReqNackPkt_(const std::vector<uint16_t> &seqVect,
                              uint32_t sourceSSRC,
                              std::vector<uint16_t> &failedSeqs) {
  int ret = 0;

  const auto &m = belongingRtpfb_.belongingRtcpHandler_.belongingPC_.rtpHandler_
                      .ssrcInfoMap_;
  auto it = m.find(sourceSSRC);
  if (it == m.end()) {
    // assert(!"we send to client firstly, SSRC should already in queue");
    // 不加密场景发包检测松，可能PC超时断开后立马又收到包，此时还未建立下行，收到端上之前的nack，用信令后不应走到这里
    return 0;
  }

  const SSRCInfo &ssrcInfo = it->second;

  for (uint16_t itemSeq : seqVect) {
    int64_t itemCycle = 0;

    if (itemSeq == ssrcInfo.biggestSeq) {
      itemCycle = ssrcInfo.biggestCycle;
    } else if (itemSeq > ssrcInfo.biggestSeq) {
      if (AheadOf(itemSeq, ssrcInfo.biggestSeq)) {
        tylog("should not recv, itemSeq=%u too newer, %s.", itemSeq,
              ssrcInfo.ToString().data());
        // assert(!"nack should not recv newer seq");
        continue;
      } else {
        if (ssrcInfo.biggestCycle <= 0) {
          tylog("should not reach here, itemSeq=%u, %s.", itemSeq,
                ssrcInfo.ToString().data());
          // assert(!"should not reach here");
          continue;
        } else {
          itemCycle = ssrcInfo.biggestCycle - 1;  // notice
        }
      }
    } else {
      if (AheadOf(itemSeq, ssrcInfo.biggestSeq)) {
        tylog("should not recv, itemSeq=%u too newer, %s.", itemSeq,
              ssrcInfo.ToString().data());
        // assert(!"nack should not recv newer seq");
        continue;
      } else {
        itemCycle = ssrcInfo.biggestCycle;
      }
    }

    PowerSeqT itemPowerSeq = itemCycle << 16 | itemSeq;
    tylog("itemCycle=%ld, itemSeq=%u, itemPowerSeq=%ld.", itemCycle, itemSeq,
          itemPowerSeq);

    const std::vector<char> *rawPacket =
        ssrcInfo.rtpSender.GetSeqPacket(itemPowerSeq);
    if (nullptr == rawPacket) {
      tylog("NOTE: nack not found packet, powerseq=%s, ssrcInfo=%s.",
            PowerSeqToString(itemPowerSeq).data(), ssrcInfo.ToString().data());

      failedSeqs.push_back(itemSeq);

      continue;
    }

    assert(!rawPacket->empty());
    // OPT: rawPacket is encrypted data, but exclude head
    DumpSendPacket(*rawPacket);

    ret = belongingRtpfb_.belongingRtcpHandler_.belongingPC_.SendToClient(
        *rawPacket);
    if (ret) {
      tylog("send to peer ret=%d, seq=%u, continue handle other nack seq", ret,
            itemSeq);

      continue;
      // return ret;
    }

    tylog("send nack packet seq=%u succ", itemSeq);
  }

  return 0;
}

// downlink
int RtcpNack::HandleNack(const RtcpHeader &chead) {
  int ret = 0;

  const uint32_t rtcpLen = chead.getRealLength();
  std::vector<uint16_t> seqVect;

  uint32_t currPos = 12;  // 调过NACK包头12个字节的长度
  const uint8_t *nackMovPointer = reinterpret_cast<const uint8_t *>(&chead);
  const RtcpHeader *pnackHead = nullptr;

  while (currPos < rtcpLen) {
    pnackHead = reinterpret_cast<const RtcpHeader *>(nackMovPointer);
    uint16_t pid = pnackHead->getNackPid();
    uint16_t blp = pnackHead->getNackBlp();

    for (int i = -1; i <= 16; i++) {
      uint16_t seqNum = pid + i + 1;
      if (i == -1 || (blp >> i) & 0x0001) {
        seqVect.push_back(seqNum);
      }
    }

    currPos += 4;
    // pid以及blp总的4个字节，读完就调过，读下一个pid以及blp
    nackMovPointer += 4;
  }

  tylog("mediaSourceSSRC=%u(0x%X), %s, nack seqs=%s.", chead.getSourceSSRC(),
        chead.getSourceSSRC(), chead.ToString().data(),
        tylib::AnyToString(seqVect).data());

  std::vector<uint16_t> failedSeqs;
  ret = SendReqNackPkt_(seqVect, chead.getSourceSSRC(), failedSeqs);
  if (ret) {
    tylog("sendReqNackPkt ret=%d", ret);

    return ret;
  }

  if (!failedSeqs.empty()) {
    tylog("warning: downlink queue not found nack req, failed seqs=%s.",
          tylib::AnyToString(failedSeqs).data());
    // should nack remote uplink client
  }

  return 0;
}

int RtcpNack::SerializeNackSend_(const std::vector<NackBlock> &nackBlokVect,
                                 uint32_t sinkSSRC, uint32_t soucreSSRC) {
  int ret = 0;

  RtcpHeader nack;
  nack.setPacketType(RtcpPacketType::kRtpFeedback);
  nack.setBlockCount(1);
  nack.setSSRC(sinkSSRC);
  nack.setSourceSSRC(soucreSSRC);
  nack.setLength(2 + nackBlokVect.size());

  if (12 + nackBlokVect.size() * 4 > 2048) {
    tylog("nack pkg len > 2048");

    return -1;
  }

  const char *head = reinterpret_cast<const char *>(&nack);
  std::vector<char> rtcpBin(head, head + 12);

  const char *body = reinterpret_cast<const char *>(&nackBlokVect[0]);
  rtcpBin.insert(rtcpBin.end(), body, body + nackBlokVect.size() * 4);

  DumpSendPacket(rtcpBin);

  ret = this->belongingRtpfb_.belongingRtcpHandler_.belongingPC_.srtpHandler_
            .ProtectRtcp(const_cast<std::vector<char> *>(&rtcpBin));
  if (ret) {
    tylog("uplink send to src client, protect rtcp ret=%d", ret);
    return ret;
  }

  ret = this->belongingRtpfb_.belongingRtcpHandler_.belongingPC_.SendToClient(
      rtcpBin);
  if (ret) {
    tylog("send to client nack rtcp ret=%d", ret);

    return ret;
  }

  return 0;
}

// uplink
int RtcpNack::CreateNackSend(const std::set<int> &lostSeqs, uint32_t localSSRC,
                             uint32_t remoteSSRC) {
  tylog("lostseqs=%s.", tylib::AnyToString(lostSeqs).data());

  int ret = 0;

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

  ret = SerializeNackSend_(nackBlokVect, localSSRC, remoteSSRC);
  if (ret) {
    tylog("serializeNack ret=%d", ret);

    return ret;
  }

  return 0;
}

}  // namespace tywebrtc