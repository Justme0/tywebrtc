// https://datatracker.ietf.org/doc/html/rfc3550

#include "rtp/rtcp/rtcp_handler.h"

#include <cassert>

#include "tylib/ip/ip.h"
#include "tylib/time/timer.h"

#include "log/log.h"
#include "pc/peer_connection.h"
#include "rtp/rtcp/rtcp_parser.h"
#include "rtp/rtp_parser.h"

extern int g_sock_fd;

RtcpHandler::RtcpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

// downlink send lost package
int RtcpHandler::SendReqNackPkt(const std::vector<uint16_t> &seqVect,
                                uint32_t sourceSSRC,
                                std::vector<uint16_t> &failedSeqs) {
  int ret = 0;

  auto it = belongingPeerConnection_.rtpHandler_.ssrcInfoMap_.find(sourceSSRC);

  // we send to client firstly, SSRC already in queue
  assert(it != belongingPeerConnection_.rtpHandler_.ssrcInfoMap_.end());

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
      tylog("NOTE: nack not found packet, powerseq=%s, %s.",
            PowerSeqToString(itemPowerSeq).data(), ssrcInfo.ToString().data());

      failedSeqs.push_back(itemSeq);

      continue;
    }

    assert(!rawPacket->empty());
    // OPT: rawPacket is encrypted data, but exclude head
    DumpSendPacket(*rawPacket);

    ret = belongingPeerConnection_.SendToClient(*rawPacket);
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

int RtcpHandler::HandleNack(const RtcpHeader &chead) {
  int ret = 0;

  uint32_t rtcpLen = (chead.getLength() + 1) * 4;
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

  tylog("mediaSourceSSRC=%u(0x%X), %s, nack seqs=%s.",
        chead.getMediaSourceSSRC(), chead.getMediaSourceSSRC(),
        chead.ToString().data(), tylib::AnyToString(seqVect).data());

  std::vector<uint16_t> failedSeqs;
  ret = SendReqNackPkt(seqVect, chead.getMediaSourceSSRC(), failedSeqs);
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

// 处理解密后的 RTCP 包
int RtcpHandler::HandleRtcpPacket(const std::vector<char> &vBufReceive) {
  // if peer doesn't exist, not handle it.
  // OPT: handle RRTR, RR, SR ...
  auto peerPC = belongingPeerConnection_.FindPeerPC();
  // if pull rtmp/srt/... stream, it's null;
  // if pull another webrtc, it's not null.

  int ret = 0;

  const char *movingBuf = vBufReceive.data();
  int rtcpLen = 0;
  size_t totalLen = 0;
  int index = 0;

  //可能是rtcp组合包
  while (true) {
    if (totalLen >= vBufReceive.size()) {
      break;
    }

    movingBuf += rtcpLen;
    const RtcpHeader *chead = reinterpret_cast<const RtcpHeader *>(movingBuf);
    rtcpLen = (chead->getLength() + 1) * 4;
    totalLen += rtcpLen;

    tylog(
        "recv number #%d rtcp=%s. rtcpLen=%u, totalLen=%zu. (input buf "
        "len=%zu)",
        index, chead->ToString().data(), rtcpLen, totalLen, vBufReceive.size());
    ++index;

    if (totalLen > vBufReceive.size()) {
      // should check why
      tylog("NOTE: totalLen=%zu > all input len=%zu, break", totalLen,
            vBufReceive.size());

      break;
    }

    // TODO: use virtual function? instead of switch
    switch (chead->packettype) {
      case RtcpPacketType::kGenericRtpFeedback: {
        switch (
            static_cast<RtcpGenericFeedbackFormat>(chead->getBlockCount())) {
          case RtcpGenericFeedbackFormat::kFeedbackNack: {
            ret = HandleNack(*chead);
            if (ret) {
              tylog("handleNack ret=%d", ret);

              return ret;
            }

            // taylor OPT: should not return, should transfer other type of
            // packet
            return 0;

            break;
          }
          case RtcpGenericFeedbackFormat::kFeedbackTCC: {
            tylog("TCC feedback rtcp pkt.");

            break;
          }
          default:
            // should only log
            assert(!"unknown rtcp");
            break;
        }
        break;
      }

      case RtcpPacketType::kPayloadSpecificFeedback: {
        // TODO: move to ToString, use virtual function?
        tylog(
            "specific feedback recv type [%s]",
            RtcpPayloadSpecificFormatToString(
                static_cast<RtcpPayloadSpecificFormat>(chead->getBlockCount()))
                .data());

        switch (
            static_cast<RtcpPayloadSpecificFormat>(chead->getBlockCount())) {
          case RtcpPayloadSpecificFormat::kRtcpPLI:
          case RtcpPayloadSpecificFormat::kRtcpSLI:
          case RtcpPayloadSpecificFormat::kRtcpFIR: {
            if (nullptr == peerPC) {
              tylog(
                  "another peerPC null(may only pull rtmp/srt/...), can not "
                  "req I frame.");

              break;
            }

            RtcpHeader *rsphead = const_cast<RtcpHeader *>(chead);
            assert(peerPC->rtpHandler_.upVideoSSRC != 0);
            rsphead->setMediaSourceSSRC(peerPC->rtpHandler_.upVideoSSRC);

            // OPT: handle other type of RTCP source ssrc
            // if (rsphead->getSourceSSRC() == kDownlinkAudioSsrc) {
            //   assert(peerPC->rtpHandler_.upAudioSSRC != 0);
            //   rsphead->setMediaSourceSSRC(peerPC->rtpHandler_.upAudioSSRC);
            // } else if (rsphead->getSourceSSRC() == kDownlinkVideoSsrc) {
            //   assert(peerPC->rtpHandler_.upVideoSSRC != 0);
            //   rsphead->setMediaSourceSSRC(peerPC->rtpHandler_.upVideoSSRC);
            // }

            break;
          }

          case RtcpPayloadSpecificFormat::kRtcpRPSI:
            break;
          case RtcpPayloadSpecificFormat::kRtcpREMB:
            break;
          default:
            // should only log
            assert(!"unknown rtcp");
            break;
        }
        break;
      }

      default:
        break;
    }
  }

  if (nullptr == peerPC) {
    tylog("another peerPC null, not transparent transfer.");

    return 0;
  }

  DumpSendPacket(vBufReceive);
  ret = peerPC->srtpHandler_.ProtectRtcp(
      const_cast<std::vector<char> *>(&vBufReceive));
  if (ret) {
    tylog("downlink protect rtcp ret=%d", ret);
    return ret;
  }

  // unvarnished transmission to peer
  ret = peerPC->SendToClient(vBufReceive);
  if (ret) {
    tylog("send ret=%d", ret);
    return ret;
  }

  return 0;
}

/*
int RtcpHandler::HandleSenderReport() {
  auto sr = std::static_pointer_cast<RTCPSenderReport>(packet);

  // Get ssrc
  uint32_t ssrc = sr->GetSSRC();

#ifndef __USE_SINGLE_THREAD__
  mutex_.lock();
#endif

  remote_send_report_map_[ssrc].emplace_front(
      SenderReportInfo{g_now_ms, sr->GetNTPTimestamp()});

  if (rtcp_observer_) {
    rtcp_observer_->OnHandleSenderReport(
        ssrc, sr->GetNTPSec(), sr->GetNTPFrac(), sr->GetRTPTimestamp());
  }

#ifndef __USE_SINGLE_THREAD__
  mutex_.unlock();
#endif

  WEBRTC_LOG_DEBUG(std::to_string(ssrc).c_str(), "g_now_ms %llu, ntp_time %llu",
                   g_now_ms, sr->GetNTPTimestamp());

  // Process all the Sender Reports
  for (uint32_t j = 0; j < sr->GetCount(); j++) {
    HandleReceiverReport(sr->GetReport(j), g_now_ms);
  }

  return 0;
}

// get lost、rtt
int RtcpHandler::HandleReceiverReport() {
  auto rr = std::static_pointer_cast<RTCPReceiverReport>(packet);

  // Process all the receiver Reports
  for (uint32_t j = 0; j < rr->GetCount(); j++) {
    HandleReceiverReport(rr->GetReport(j), g_now_ms);
  }

  return 0;
}

int RtcpHandler::HandleReceiverReport() {
  uint32_t ssrc = report->GetSSRC();

  uint32_t last_sr = report->GetLastSR();
  uint32_t dlsr = report->GetDelaySinceLastSR();

  auto now_ntp = MsToNtp(g_now_ms);

  uint32_t middle_ntp = (now_ntp.seconds() & 0x0000FFFF) << 16;
  middle_ntp |= (now_ntp.fractions() & 0xFFFF0000) >> 16;

  // RTT in 1/2^16 second fractions.
  uint32_t middle_ntp_delta{0};

  // remote maybe not received sr, lastrt dlsr maybe zero
  if (last_sr && dlsr && (middle_ntp > dlsr + last_sr)) {
    middle_ntp_delta = middle_ntp - dlsr - last_sr;
  }

  // RTT in milliseconds.
  float rtt_ms = static_cast<float>(middle_ntp_delta >> 16) * 1000;
  rtt_ms += (static_cast<float>(middle_ntp_delta & 0x0000FFFF) / 65536) * 1000;

  rtcp_observer_->OnReceiverReport(ssrc, static_cast<uint32_t>(rtt_ms),
                                   report->GetFactionLost(),
                                   report->GetJitter(), report->GetLostCount());

  return 0;
}

int RtcpHandler::HandleRtpFeedbackReport() {
  // Get feedback packet
  std::vector<uint16_t> seq_vect;

  auto fb = std::static_pointer_cast<RTCPRTPFeedback>(packet);
  // Get SSRC for media
  uint32_t ssrc = fb->GetSenderSSRC();

  // Check feedback type
  switch (fb->GetFeedbackType()) {
    case RTCPRTPFeedback::NACK:
      for (uint8_t i = 0; i < fb->GetFieldCount(); i++) {
        // Get field
        auto field = fb->GetField<RTCPRTPFeedback::NACKField>(i);
        seq_vect.push_back(field->pid);

        // Check each bit of the mask
        for (uint8_t i = 0; i < 16; i++) {
          // Check it bit is present to rtx the packets
          if ((field->blp >> i) & 1) {
            uint16_t seq = field->pid + i + 1;
            seq_vect.push_back(seq);
          }
        }
      }
      rtcp_observer_->OnHandleNack(seq_vect, fb->GetMediaSSRC());
      break;
    case RTCPRTPFeedback::TempMaxMediaStreamBitrateRequest:
      break;
    case RTCPRTPFeedback::TempMaxMediaStreamBitrateNotification:
      break;
    case RTCPRTPFeedback::TransportWideFeedbackMessage:
      for (uint8_t i = 0; i < fb->GetFieldCount(); i++) {
        // Get field
        auto field = fb->GetField<TwccPacket>(i);
        const TwccPacket &twcc_packet = *(field.get());
        rtcp_observer_->OnHandleTwcc(twcc_packet);
      }
      break;
  }

  return 0;
}

int RtcpHandler::HandlePayloadFeedbackReport() {
  // Get feedback packet
  auto fb = std::static_pointer_cast<RTCPPayloadFeedback>(packet);
  // Get SSRC for media
  uint32_t ssrc = fb->GetMediaSSRC();

  // Check feedback type
  switch (fb->GetFeedbackType()) {
    case RTCPPayloadFeedback::PictureLossIndication:
    case RTCPPayloadFeedback::FullIntraRequest: {
      rtcp_observer_->OnRequestLocalIFrame(ssrc);
      break;
    }
    case RTCPPayloadFeedback::SliceLossIndication:
      break;
    case RTCPPayloadFeedback::ReferencePictureSelectionIndication:
      break;
    case RTCPPayloadFeedback::TemporalSpatialTradeOffRequest:
      break;
    case RTCPPayloadFeedback::TemporalSpatialTradeOffNotification:
      break;
    case RTCPPayloadFeedback::VideoBackChannelMessage:
      break;
    case RTCPPayloadFeedback::ApplicationLayerFeeedbackMessage:
      // For all message fields
      for (uint8_t i = 0; i < fb->GetFieldCount(); i++) {
        // Get feedback
        auto msg =
            fb->GetField<RTCPPayloadFeedback::ApplicationLayerFeeedbackField>(
                i);
        // Get size and payload
        uint32_t len = msg->GetLength();
        const uint8_t *payload = msg->GetPayload();
        // Check if it is a REMB
        if (len > 8 && payload[0] == 'R' && payload[1] == 'E' &&
            payload[2] == 'M' && payload[3] == 'B') {
          // Get SSRC count
          uint8_t num = payload[4];
          // GEt exponent
          uint8_t exp = payload[5] >> 2;
          uint32_t mantisa = payload[5] & 0x03;
          mantisa = mantisa << 8 | payload[6];
          mantisa = mantisa << 8 | payload[7];
          // Get bitrate
          uint32_t bitrate = mantisa << exp;
          // For each
          for (uint32_t i = 0; i < num; ++i) {
            // Check length
            if (len < 8 + 4 * i + 4)
              // wrong format
              break;
            // Get ssrc
            uint32_t target = get4(payload, 8 + 4 * i);
            // Get media
            rtcp_observer_->OnRembBitrate(target, bitrate);
          }
        }
      }
      break;
  }

  return 0;
}

int RtcpHandler::HandleXrExtendReport() {
  auto fb = std::static_pointer_cast<RTCPXr>(packet);
  WEBRTC_LOG_DEBUG("", "recv xr extent report");
  for (uint8_t i = 0; i < fb->GetFieldCount(); i++) {
    // Get field
    // auto field = fb->GetField<RTCPXr::DLRRField>(i);
    // for (auto& dlrr : field->dlrrinfo) {
    //     rtcp_observer_->OnHandleXr(dlrr.ssrc, dlrr.lrr, dlrr.dlrr);
    // }
    auto field = fb->GetField(i);
    if (field->GetType() == RTCPXr::RRT) {
      auto rrtr = std::static_pointer_cast<RTCPXr::RRTRField>(field);
      last_receive_rrtr_time_ms_ = g_now_ms;
      last_rrtr_ntp_time_ = rrtr->GetNtpTime();
      rrtr_ssrc_ = rrtr->GetSSRC();
    }
  }
  return 0;
}

void RtcpHandler::HandleTwcc(RtcpHeader *chead) {
    uint32_t rtcp_len = (chead->getLength() + 1) * 4;
    uint8_t* data = reinterpret_cast<uint8_t *>(chead);
    uint16_t len = rtcp_len;

    TwccPacket twcc_packet;
    if (twcc_packet.Decode(data, len) != 0) {
        WEBRTC_LOG_ERROR(stream_id_.c_str(), "parse twcc packet failed");
        return;
    }

    if (rtcp_observer_) {
        rtcp_observer_->OnHandleTwcc(twcc_packet);
    }

    return;
}*/

/*
uint32_t RtcpHandler::CreateSenderReport(RTCPData::shared data, char *out,
                                         int max_len) {
  auto p = std::static_pointer_cast<RTCPSenderReportData>(data);

  auto rtcp = std::make_shared<RTCPCompoundPacket>();

  auto sr = std::make_shared<RTCPSenderReport>();

  // Append data
  sr->SetSSRC(p->local_ssrc_);
  sr->SetTimestamp(p->now_time_ms_ * 1000);
  sr->SetRtpTimestamp(p->last_rtp_ts_);
  sr->SetOctectsSent(p->total_bytes_);
  sr->SetPacketsSent(p->total_pkg_num_);

  rtcp->AddPacket(sr);

  if (last_receive_rrtr_time_ms_ != 0) {
    auto packetXr = std::make_shared<RTCPXr>(p->local_ssrc_);
    uint64_t coreNtp =
        static_cast<uint64_t>(MsToNtp(p->now_time_ms_)) -
        static_cast<uint64_t>(MsToNtp(last_receive_rrtr_time_ms_));
    uint32_t dlrrNtp = CompactNtp(NtpTime(coreNtp));
    auto dllrpacket = std::make_shared<RTCPXr::DLRRField>(
        rrtr_ssrc_, CompactNtp(NtpTime(last_rrtr_ntp_time_)), dlrrNtp);
    packetXr->AddField(dllrpacket);
    rtcp->AddPacket(packetXr);
    last_receive_rrtr_time_ms_ = 0;
  }

  return rtcp->Serialize(reinterpret_cast<uint8_t *>(out), max_len);
}

uint32_t RtcpHandler::CreateXr(std::vector<char>& o_rtcpPacketBin) {
  auto p = std::static_pointer_cast<RTCPXrData>(data);

  auto rtcp = std::make_shared<RTCPCompoundPacket>();

  auto rrtr = std::make_shared<RTCPXr::RRTRField>(p->ssrc_);
  rrtr->SetNtp(p->ntp_);

  auto xr = std::make_shared<RTCPXr>();
  xr->SetSSRC(p->ssrc_);
  xr->AddField(rrtr);

  rtcp->AddPacket(xr);

  return rtcp->Serialize(reinterpret_cast<uint8_t *>(out), max_len);
}

// create rr、remb
uint32_t RtcpHandler::CreateReceiverReport(RTCPData::shared data, char *out,
                                           int max_len) {
  auto p = std::static_pointer_cast<RTCPReceiverReportData>(data);

  // Create rtcp sender retpor
  auto rtcp = RTCPCompoundPacket::Create();

  // Create receiver report for normal stream
  auto rr = rtcp->CreatePacket<RTCPReceiverReport>(p->local_ssrc_);

  // Create report, recv new rtp
  if (p->recv_pkg_num_ != p->last_recv_pkg_num_) {
    // Create report
    RTCPReport::shared report = std::make_shared<RTCPReport>();

    // Set SSRC of incoming rtp stream
    report->SetSSRC(p->remote_ssrc_);

    uint32_t delay = 0;
    uint64_t last_sender_ntp_time = 0;

#ifndef __USE_SINGLE_THREAD__
    mutex_.lock();
#endif
    if (remote_send_report_map_.find(p->remote_ssrc_) !=
        remote_send_report_map_.end()) {
      // find the first sr < now_time
      for (auto sr : remote_send_report_map_[p->remote_ssrc_]) {
        if (sr.time_ms < p->now_time_ms_) {
          delay = p->now_time_ms_ - sr.time_ms;
          last_sender_ntp_time = sr.ntp_time;
          break;
        }
      }

      if (delay > 0) {
        // at least save one sr
        for (auto it = remote_send_report_map_[p->remote_ssrc_].begin() + 1;
             it != remote_send_report_map_[p->remote_ssrc_].end();) {
          if (it->time_ms < p->now_time_ms_) {
            it = remote_send_report_map_[p->remote_ssrc_].erase(it);
          } else {
            ++it;
          }
        }
      }
    }
    WEBRTC_LOG_DEBUG("", "ssrc:%u, delay:%u, sr_ntp_time:%llu, now_time %llu, ",
                     p->remote_ssrc_, delay, last_sender_ntp_time,
                     p->now_time_ms_);

#ifndef __USE_SINGLE_THREAD__
    mutex_.unlock();
#endif

    report->SetLostCount(p->lost_);

    // Get time and update it
    report->SetDelaySinceLastSRMilis(delay);
    // The middle 32 bits out of 64 in the NTP timestamp (as explained in
    // Section 4)
    // received as part of the most recent RTCP sender report (SR) packet from
    // source SSRC_n.
    // If no SR has been received yet, the field is set to zero.
    // Other data
    report->SetLastSR(last_sender_ntp_time >> 16);
    report->SetFractionLost(p->frac_lost_);
    report->SetLastJitter(p->jitter_);
    report->SetLastSeqNum(p->extended_seq_);

    // If got anything
    if (report) {
      rr->AddReport(report);
    }

    if (p->rbe_bitrate_ > 0) {
      // Remb
      // SSRC of media source (32 bits):  Always 0; this is the same convention
      // as in [RFC5104] section 4.2.2.2 (TMMBN).
      auto remb = rtcp->CreatePacket<RTCPPayloadFeedback>(
          RTCPPayloadFeedback::ApplicationLayerFeeedbackMessage, p->local_ssrc_,
          uint16_t(0));
      // Send estimation
      std::list<uint32_t> ssrcs(1, p->remote_ssrc_);
      remb->AddField(
          RTCPPayloadFeedback::ApplicationLayerFeeedbackField::
              CreateReceiverEstimatedMaxBitrate(ssrcs, p->rbe_bitrate_));
    }
  }

  return rtcp->Serialize(reinterpret_cast<uint8_t *>(out), max_len);
}



uint32_t RtcpHandler::CreateBye(RtcpByeInfo &bye, std::vector<char>&
o_rtcpPacketBin) {
  // Create rtcp sender retpor
  auto rtcp = RTCPCompoundPacket::Create();
  auto bye_pkt = rtcp->CreatePacket<RTCPBye>(bye.ssrcs, bye.reason.c_str());

  // Serialize it
  return rtcp->Serialize(reinterpret_cast<uint8_t *>(out), max_len);
}

// in param is poor, should re-design
// temp no use
int RtcpHandler::CreateRtcpPacket(RtcpPacketType packetType, FeedbackType
feedbackType, std::vector<char>& o_rtcpPacketBin) {
  switch (packetType) {
    case RtcpPacketType::kSenderReport: {
      return CreateSenderReport(data, out, max_len);
    }
    case RtcpPacketType::kReceiverReport: {
      return CreateReceiverReport(data, out, max_len);
    }
    case RtcpPacketType::kRTPFeedback: {
      if (feedbackType == RtcpPacketType::NACK) {
        // return CreateNackReport(o_rtcpPacketBin);
      }
    }
    case RtcpPacketType::kPayloadSpecificFeedback: {
      // if (data->PayloadType() == RTCPPayloadFeedback::PictureLossIndication)
      // {
      //   return CreatePLIReport(data, out, max_len);
      // }
    }
    case RtcpPacketType::XrExtend: {
      return CreateXr(data, out, max_len);
    }

    default:
      tylog("we don't deal with RTCP type=%d[%s]", type,
            RtcpPacketTypeToString(type).data());
      // should not return err?
      return -1;
  }
  return 0;
}

uint64_t RtcpHandler::GetTimeStamps(uint32_t local_ssrc, uint32_t remote_ssrc,
                                    uint32_t rtp_timestamp) {
  return 0;
}
*/

int RtcpHandler::SerializeNackSend_(const std::vector<NackBlock> &nackBlokVect,
                                    uint32_t sinkSSRC, uint32_t soucreSSRC) {
  int ret = 0;

  RtcpHeader nack;
  nack.setPacketType(RtcpPacketType::kGenericRtpFeedback);
  nack.setBlockCount(1);
  nack.setSSRC(sinkSSRC);
  nack.setMediaSourceSSRC(soucreSSRC);
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

  ret = this->belongingPeerConnection_.srtpHandler_.ProtectRtcp(
      const_cast<std::vector<char> *>(&rtcpBin));
  if (ret) {
    tylog("uplink send to src client, protect rtcp ret=%d", ret);
    return ret;
  }

  ret = this->belongingPeerConnection_.SendToClient(rtcpBin);
  if (ret) {
    tylog("send to client nack rtcp ret=%d", ret);

    return ret;
  }

  return 0;
}

int RtcpHandler::CreateNackReportSend(const std::set<int> &lostSeqs,
                                      uint32_t localSSRC, uint32_t remoteSSRC) {
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

int RtcpHandler::CreatePLIReportSend(uint32_t ssrc, uint32_t sourceSSRC) {
  int ret = 0;

  tylog("create PLI, localSSRC=%u, remoteSSRC=%u.", ssrc, sourceSSRC);

  RtcpHeader pli;
  pli.setPacketType(RtcpPacketType::kPayloadSpecificFeedback);
  pli.setBlockCount(static_cast<uint8_t>(RtcpPayloadSpecificFormat::kRtcpPLI));
  pli.setSSRC(ssrc);
  pli.setMediaSourceSSRC(sourceSSRC);
  pli.setLength(2);

  const char *head = reinterpret_cast<const char *>(&pli);
  const int len = (pli.getLength() + 1) * 4;
  assert(len == 12);                            // head len
  std::vector<char> rtcpBin(head, head + len);  // can use string view

  DumpSendPacket(rtcpBin);

  ret = this->belongingPeerConnection_.srtpHandler_.ProtectRtcp(
      const_cast<std::vector<char> *>(&rtcpBin));
  if (ret) {
    tylog("uplink send to src client, protect rtcp ret=%d", ret);

    return ret;
  }

  ret = this->belongingPeerConnection_.SendToClient(rtcpBin);
  if (ret) {
    tylog("send to client nack rtcp ret=%d", ret);

    return ret;
  }

  tylog("send PLI succ, ssrc=%u, source ssrc=%u.", ssrc, sourceSSRC);

  return 0;
}
