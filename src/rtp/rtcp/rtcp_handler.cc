#include "rtp/rtcp/rtcp_handler.h"

#include <cassert>

#include "tylib/ip/ip.h"
#include "tylib/time/timer.h"

#include "log/log.h"
#include "pc/peer_connection.h"
#include "rtp/rtcp/rtcp_parser.h"

extern int g_sock_fd;

RtcpHandler::RtcpHandler(PeerConnection &pc) : belongingPeerConnection_(pc) {}

int RtcpHandler::HandleRtcpPacket(const std::vector<char> &vBufReceive) {
  int ret = 0;

  // const RtcpHeader &rtcpHeader =
  //     *reinterpret_cast<const RtcpHeader *>(vBufReceive.data());

  /*
    RtcpHeader *chead = reinterpret_cast<RtcpHeader *>(pPackage);
    if (!chead->isRtcp()) {
      tylog("pkg is not rtcp, pt=%d", chead->packettype);
      return -1;
    }

    unsigned int subType = 0;
    STVideoUserInfo *pWatcherUser = nullptr;

    // 行使用远端ssrc判断
    if (m_pTWebRTCUserInfo->ullTinyId == m_pc->ullTinyId) {
      if (chead->getSSRC() == m_pc->tVideoUserInfo[0].uiRemoteSsrc)  //视频SR包
      {
        pWatcherUser = &m_pc->tVideoUserInfo[0];
        subType = DT_BIGVIDIO;
      } else if (chead->getSSRC() == m_pc->tVideoUserInfo[1].uiRemoteSsrc) {
        pWatcherUser = &m_pc->tVideoUserInfo[1];
      } else if (chead->getSSRC() == m_pc->tAudioInfo.uiRemoteSsrc)  //音频SR包
      {
        subType = DT_AUDIO;
      }
    } else {
      if (chead->getSourceSSRC() == m_pc->tVideoUserInfo[0].uiLocalSsrc) {
        subType = DT_BIGVIDIO;
        pWatcherUser = &m_pc->tVideoUserInfo[0];
      } else if (chead->getSourceSSRC() == m_pc->tVideoUserInfo[1].uiLocalSsrc)
    { pWatcherUser = &m_pc->tVideoUserInfo[1]; } else if (chead->getSourceSSRC()
    == m_pc->tAudioInfo.uiLocalSsrc) { subType = DT_AUDIO;
      }
    }

    if (0 == subType) {
      tylog(
          "Rtcp packettype:%u BlockCount:%u subType:%u\r\n Rtcp[SSrc:%u "
          "SrcSSRC:%u] \r\nVid[RemSsrc:%u LocSsrc:%u] \r\nAud[RemSsrc:%u "
          "LocSsrc:%u]",
          chead->packettype, chead->getBlockCount(), subType, chead->getSSRC(),
          chead->getSourceSSRC(), m_pc->tVideoUserInfo[0].uiRemoteSsrc,
          m_pc->tVideoUserInfo[0].uiLocalSsrc, m_pc->tAudioInfo.uiRemoteSsrc,
          m_pc->tAudioInfo.uiLocalSsrc);
    }

    char *movingBuf = pPackage;
    int rtcpLen = 0;
    int totalLen = 0;
    while (true)  //可能是rtcp组合包
    {
      if (totalLen >= iLen) break;

      movingBuf += rtcpLen;
      chead = reinterpret_cast<RtcpHeader *>(movingBuf);
      rtcpLen = (chead->getLength() + 1) * 4;
      totalLen += rtcpLen;

      tylog("rtcpLen[%u], totalLen[%u], iLen[%u]", rtcpLen, totalLen,
                         iLen);

      if (totalLen > iLen) break;

      if (chead->packettype == RTCP_RTP_Feedback_PT)  // NACK  RFC3550
      {
        if (chead->getBlockCount() == 1) {
          HandleNack(chead);
        } else if (chead->getBlockCount() == RTCP_AFB) {
          // REMB只会有在房间只有两个web的时候透传
          if (IsByPassMod()) {
            RelayRtcpReq(movingBuf, rtcpLen, subType);
          }

          tylog(
              "REMB[Recv]:  BitRate[%lu] packettype:%u BlockCount:%u "
              "subType:%u\r\n Rtcp[SSrc:%u SrcSSRC:%u] \r\nVid[RemSsrc:%u "
              "LocSsrc:%u] \r\nAud[RemSsrc:%u LocSsrc:%u]",
              chead->getREMBBitRate(), chead->packettype,
    chead->getBlockCount(), subType, chead->getSSRC(), chead->getSourceSSRC(),
              m_pc->tVideoUserInfo[0].uiRemoteSsrc,
              m_pc->tVideoUserInfo[0].uiLocalSsrc,
    m_pc->tAudioInfo.uiRemoteSsrc, m_pc->tAudioInfo.uiLocalSsrc);
        }

      } else if (chead->packettype == RTCP_PS_Feedback_PT) {
        // I帧申请透传
        if ((RTCP_PLI_FMT == chead->getBlockCount()) ||
            (RTCP_SLI_FMT == chead->getBlockCount()) ||
            (RTCP_FIR_FMT == chead->getBlockCount())) {
          if (m_pTWebRTCUserInfo->ullTinyId == m_pc->ullTinyId) {
            continue;
          }

          tylog(
              "PLI: MediaMode:%u ClientType:%u Rtcp packettype:%u BlockCount:%u
    " "subType:%u\r\n Rtcp[SSrc:%u SrcSSRC:%u] \r\nVid[RemSsrc:%u " "LocSsrc:%u]
    \r\nAud[RemSsrc:%u LocSsrc:%u]", m_pTWebRTCUserInfo->MediaMode,
    m_pc->ClientType, chead->packettype, chead->getBlockCount(), subType,
    chead->getSSRC(), chead->getSourceSSRC(),
    m_pc->tVideoUserInfo[0].uiRemoteSsrc, m_pc->tVideoUserInfo[0].uiLocalSsrc,
    m_pc->tAudioInfo.uiRemoteSsrc, m_pc->tAudioInfo.uiLocalSsrc);

          if (!pWatcherUser) {
            continue;
          }

          // TODO 位置修改
          pWatcherUser->PLICnt++;
          if (m_pc->ClientType != USER_TYPE_WEBRTC) {
            // TODO 位置修改 急速模式下sdk
            tylog("rtcp from web call RequestPeerIDRFrame");
            pWatcherUser->ReqSdkIFrFlag++;
            if (pWatcherUser->ReqSdkIFrFlag == 0) {
              pWatcherUser->ReqSdkIFrFlag = 1;
            }
            m_pTWebRTCUserInfo->RequestPeerIDRFrame(m_pc, pWatcherUser);
          } else if (m_pc->ClientType == USER_TYPE_WEBRTC) {
            // TODO 位置修改 急速模式下的web，非急速模式下的web
            pWatcherUser->ReqICnt++;
            // RelayRtcpReq(movingBuf,rtcpLen, subType);
            tylog("rtcp from web call RequestPeerIDRFrame");
            m_pTWebRTCUserInfo->RequestPeerIDRFrame(m_pc, pWatcherUser, true);
          }
        }
      } else if (chead->packettype == RTCP_Sender_PT)  // SR
      {
        //广播发送报告广播
        if (IsByPassMod()) {
          BroadcastRtcpReq(movingBuf, rtcpLen, subType);
        }

        tylog("[RTCP_Sender_PT]");
        HandleSR(chead);
      } else if (chead->packettype == RTCP_Receiver_PT)  // RR
      {
        //接收报告只会有在房间只有两个web的时候透传
        if (IsByPassMod()) {
          RelayRtcpReq(movingBuf, rtcpLen, subType);
        }

        tylog("[RTCP_Receiver_PT]");
        HandleRR(chead);
      } else if (chead->packettype == RTCP_XR_PT)  // XR  rfc3611
      {
        tylog("[ExtendedReport]");
        //广播发送报告广播
        if (IsByPassMod()) {
          BroadcastRtcpReq(movingBuf, rtcpLen, subType);
        }
        HandleXR(chead);
      }
    }
    */

  // taylor origin:

  // downlink

  // RtcpHeader &downlinkRtcpHeader = const_cast<RtcpHeader &>(rtcpHeader);

  /*
    if (mediaType == kMediaTypeAudio) {
      const int kDownlinkAudioSsrc = 16854838;  // taylor to make dynamic
      downlinkRtpHeader.setSSRC(kDownlinkAudioSsrc);

      const int kDownlinkAudioPayloadType = 111;
      downlinkRtpHeader.setPayloadType(kDownlinkAudioPayloadType);
    } else {
      const int kDownlinkVideoSsrc = 33697348;  // taylor to make dynamic
      downlinkRtpHeader.setSSRC(kDownlinkVideoSsrc);

      const int kDownlinkVideoPayloadType = 125;  // H.264
      downlinkRtpHeader.setPayloadType(kDownlinkVideoPayloadType);
    }
    */

  const char *movingBuf = vBufReceive.data();
  int rtcpLen = 0;
  size_t totalLen = 0;
  int index = 0;
  while (true)  //可能是rtcp组合包
  {
    if (totalLen >= vBufReceive.size()) break;

    movingBuf += rtcpLen;
    const RtcpHeader *chead = reinterpret_cast<const RtcpHeader *>(movingBuf);
    tylog("recv number %d rtcp=%s", index++, chead->ToString().data());
    rtcpLen = (chead->getLength() + 1) * 4;
    totalLen += rtcpLen;

    tylog("rtcpLen[%u], totalLen[%zu], input buf len[%zu]", rtcpLen, totalLen,
          vBufReceive.size());

    if (totalLen > vBufReceive.size()) break;

    switch (chead->packettype) {
      case RtcpPacketType::kSenderReport: {
        tylog("[RTCP_SenderReport_PT]");
        break;
      }

      case RtcpPacketType::kReceiverReport: {
        tylog("[RTCP_ReceiverReport_PT]");
        break;
      }
      case RtcpPacketType::kGenericRtpFeedback: {
        switch (static_cast<RtcpFeedbackFormat>(chead->getBlockCount())) {
          case RtcpFeedbackFormat::kFeedbackNack: {
            // audio have no nack?
            RtcpHeader *rsphead = const_cast<RtcpHeader *>(chead);

            if (rsphead->getSourceSSRC() == kDownlinkAudioSsrc) {
              assert(g_UplinkAudioSsrc != 0);
              rsphead->setSourceSSRC(g_UplinkAudioSsrc);
            } else if (rsphead->getSourceSSRC() == kDownlinkVideoSsrc) {
              assert(g_UplinkVideoSsrc != 0);
              rsphead->setSourceSSRC(g_UplinkVideoSsrc);
            }

            break;
          }
          case RtcpFeedbackFormat::kFeedbackTCC: {
            break;
          }
        }
        break;
      }

      case RtcpPacketType::kPayloadSpecificFeedback: {
        switch (
            static_cast<RtcpPayloadSpecificFormat>(chead->getBlockCount())) {
          case RtcpPayloadSpecificFormat::kRtcpPLI:
          case RtcpPayloadSpecificFormat::kRtcpSLI:
          case RtcpPayloadSpecificFormat::kRtcpFIR: {
            RtcpHeader *rsphead = const_cast<RtcpHeader *>(chead);
            assert(g_UplinkVideoSsrc != 0);
            rsphead->setSourceSSRC(g_UplinkVideoSsrc);

            break;
          }

          case RtcpPayloadSpecificFormat::kRtcpRPSI:
            break;
          case RtcpPayloadSpecificFormat::kRtcpREMB:
            break;
        }
        break;
      }
      case RtcpPacketType::kXrExtend: {
        tylog("[RTCP_ExtendReport_PT]");
        break;
      }

      default:
        break;
    }
  }

  ret = this->belongingPeerConnection_.srtpHandler_.ProtectRtcp(
      const_cast<std::vector<char> *>(&vBufReceive));
  if (ret) {
    tylog("downlink protect rtcp ret=%d", ret);
    return ret;
  }

  sockaddr_in addr =
      tylib::ConstructSockAddr(this->belongingPeerConnection_.clientIP_,
                               this->belongingPeerConnection_.clientPort_);
  ssize_t sendtoLen = sendto(g_sock_fd, vBufReceive.data(), vBufReceive.size(),
                             0, reinterpret_cast<struct sockaddr *>(&addr),
                             sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    return -1;
  }
  tylog("sendto reply succ buf size=%ld, ip=%s, port=%d.", sendtoLen,
        belongingPeerConnection_.clientIP_.data(),
        belongingPeerConnection_.clientPort_);

  return 0;
  /*
    for (uint32_t i = 0; i < rtcp->GetPacketCount(); i++) {
      auto packet = rtcp->GetPacket(i);
      packet->Dump();
      switch (packet->GetType()) {
        case RtcpPacketType::kFullIntraRequest: {
          break;
        }
        case RtcpPacketType::kExtendedJitterReport: {
          break;
        }
        case RtcpPacketType::kSenderReport: {
          HandleSenderReport();
          break;
        }
        case RtcpPacketType::kReceiverReport: {
          HandleReceiverReport();
          break;
        }
        case RtcpPacketType::kSDES: {
          break;
        }
        case RtcpPacketType::kBye: {
          rtcp_observer_->OnHandleBye();
          break;
        }
        case RtcpPacketType::kApp: {
          break;
        }
        case RtcpPacketType::kGenericRtpFeedback: {
          HandleRtpFeedbackReport();
          break;
        }
        case RtcpPacketType::kPayloadSpecificFeedback: {
          HandlePayloadFeedbackReport();
          break;
        }
        case RtcpPacketType::kXrExtend: {
          HandleXrExtendReport();
          break;
        }

        default: {
          tylog("unknown rtcp packet type=%d", packet->GetType());
          // should not assert if hacker sends a packet
          assert(!"unknown rtcp packet type");
        }
      }
    }
    */
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

uint32_t RtcpHandler::CreatePLIReport(RTCPData::shared data, char *out,
                                      int max_len) {
  auto p = std::static_pointer_cast<RTCPPLIReportData>(data);

  // Create rtcp sender retpor
  auto rtcp = RTCPCompoundPacket::Create();

  // Add to rtcp
  rtcp->CreatePacket<RTCPPayloadFeedback>(
      RTCPPayloadFeedback::PictureLossIndication, p->local_ssrc_,
      p->remote_ssrc_);

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