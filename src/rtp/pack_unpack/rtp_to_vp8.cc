// from
// https://source.chromium.org/chromium/chromium/src/+/main:third_party/webrtc/modules/rtp_rtcp/source/video_rtp_depacketizer_vp8.cc

#include "src/rtp/pack_unpack/rtp_to_vp8.h"

#include "src/log/log.h"
#include "src/rtp/rtp_handler.h"

namespace tywebrtc {

static int ParseVP8Descriptor(RTP_HEADER_INFO_VP8* vp8, const void* void_data,
                              int data_length) {
  const uint8_t* data = static_cast<const uint8_t*>(void_data);
  if (data_length <= 0) {
    return -1;
  }
  int parsed_bytes = 0;
  // Parse mandatory first byte of payload descriptor.
  bool extension = (*data & 0x80) ? true : false;             // X bit
  vp8->NonReference = (*data & 0x20) ? true : false;          // N bit
  vp8->BeginningOfPartition = (*data & 0x10) ? true : false;  // S bit
  vp8->PartitionId = (*data & 0x0F);                          // PartID field

  data++;
  parsed_bytes++;
  data_length--;

  if (!extension) {
    return parsed_bytes;
  }

  if (data_length == 0) return -1;
  // Optional X field is present.
  bool has_picture_id = (*data & 0x80) ? true : false;   // I bit
  bool has_tl0_pic_idx = (*data & 0x40) ? true : false;  // L bit
  bool has_tid = (*data & 0x20) ? true : false;          // T bit
  bool has_key_idx = (*data & 0x10) ? true : false;      // K bit

  // Advance data and decrease remaining payload size.
  data++;
  parsed_bytes++;
  data_length--;

  if (has_picture_id) {
    if (data_length == 0) return -1;

    vp8->PictureId = (*data & 0x7F);
    if (*data & 0x80) {
      data++;
      parsed_bytes++;
      if (--data_length == 0) return -1;
      // PictureId is 15 bits
      vp8->PictureId = (vp8->PictureId << 8) + *data;
    }
    data++;
    parsed_bytes++;
    data_length--;
  }

  if (has_tl0_pic_idx) {
    if (data_length == 0) return -1;

    vp8->Tl0PicIdx = *data;
    data++;
    parsed_bytes++;
    data_length--;
  }

  if (has_tid || has_key_idx) {
    if (data_length == 0) return -1;

    if (has_tid) {
      vp8->TemporalIdx = ((*data >> 6) & 0x03);
      vp8->LayerSync = (*data & 0x20) ? true : false;  // Y bit
    }
    if (has_key_idx) {
      vp8->KeyIdx = *data & 0x1F;
    }
    data++;
    parsed_bytes++;
    data_length--;
  }
  return parsed_bytes;
}

// now not be called, but useful
bool RtpDepacketizerVp8::IsIdrFrame(char* buf, int buf_len) {
  if (buf == nullptr || buf_len <= kRtpHeaderLenByte) {
    return false;
  }

  RtpHeader* h = reinterpret_cast<RtpHeader*>(buf);
  if (h->getHeaderLength() >= buf_len) {
    tylog("[GetRtpExtOffset] err %d", h->getHeaderLength());
    return false;
  }

  int PayloadLen = buf_len - h->getHeaderLength();
  RTP_HEADER_INFO_VP8 RtpHeadInfoVp8;
  RtpHeadInfoVp8.NonReference = false;
  RtpHeadInfoVp8.PictureId = NO_PIC_IDX;
  RtpHeadInfoVp8.Tl0PicIdx = NO_TL0_PIC_IDX;
  RtpHeadInfoVp8.TemporalIdx = NO_TEMPORAL_IDX;
  RtpHeadInfoVp8.LayerSync = false;
  RtpHeadInfoVp8.KeyIdx = NO_KEY_IDX;
  RtpHeadInfoVp8.PartitionId = 0;
  RtpHeadInfoVp8.BeginningOfPartition = false;
  const int DescriptorSize = ParseVP8Descriptor(
      &RtpHeadInfoVp8, (const uint8_t*)buf + h->getHeaderLength(), PayloadLen);
  if (DescriptorSize < 0) {
    tylog("parseVP8Descriptor ret=%d, HeadLen %d,PayloadLen %d", DescriptorSize,
          h->getHeaderLength(), PayloadLen);
    return false;
  }
  if (RtpHeadInfoVp8.PartitionId > 8) {
    // Weak check for corrupt payload_data: PartID MUST NOT be larger than 8.
    tylog(
        "Weak check for corrupt payload_data: PartID MUST NOT be "
        "larger than 8");
    return false;
  }

  bool IsFirstPacket =
      RtpHeadInfoVp8.BeginningOfPartition && RtpHeadInfoVp8.PartitionId == 0;
  int Vp8PayloadSize = PayloadLen - DescriptorSize;
  if (Vp8PayloadSize == 0) {
    tylog("Empty vp8 payload");
    return false;
  }
  const uint8_t* Vp8Payload =
      (const uint8_t*)buf + h->getHeaderLength() + DescriptorSize;
  // Read P bit from payload header (only at beginning of first partition).
  if (!(IsFirstPacket && (*Vp8Payload & 0x01) == 0)) {
    tylog("is not key frame");
    return false;
  }

  return true;
}

RtpDepacketizerVp8::RtpDepacketizerVp8(RtpHandler& rtpHandler)
    : belongingRtpHandler_(rtpHandler) {
  Init();
  decoder = NULL;
  encoder = NULL;
  // m_pFastUpDate = VIDEO_RTP_NULL;
  is_key_frame = false;
  encoder_width = 0;
  encoder_height = 0;
  m_pfOutfpH264 = NULL;
  last_receive_frame_time = 0;
}

RtpDepacketizerVp8::~RtpDepacketizerVp8() {
  if (decoder) {
    delete decoder;
    decoder = NULL;
  }

  if (encoder) {
    delete encoder;
    encoder = NULL;
  }

  // if (m_pfOutfpH264) {
  //   WriteH264FileStop(m_pfOutfpH264);
  // }
}

void RtpDepacketizerVp8::InitRTPVideoHeaderVP8() {
  m_RtpHeadInfoVp8.NonReference = false;
  m_RtpHeadInfoVp8.PictureId = NO_PIC_IDX;
  m_RtpHeadInfoVp8.Tl0PicIdx = NO_TL0_PIC_IDX;
  m_RtpHeadInfoVp8.TemporalIdx = NO_TEMPORAL_IDX;
  m_RtpHeadInfoVp8.LayerSync = false;
  m_RtpHeadInfoVp8.KeyIdx = NO_KEY_IDX;
  m_RtpHeadInfoVp8.PartitionId = 0;
  m_RtpHeadInfoVp8.BeginningOfPartition = false;
}

void RtpDepacketizerVp8::Init() {
  InitRTPVideoHeaderVP8();

  m_RawDataLen = 0;

  memset((void*)m_RtpHeadVp8, 0, VP8_MAX_PAYLOAD_HEAD_LEN);

  memset((void*)&m_RtpHeader, 0, sizeof(RtpHeader));
  m_RtpHeader.setVersion(2);
  memset((void*)&m_UnPackParams, 0, sizeof(VIDEO_VP8_UNPACK_PARAMS));

  m_UnPackParams.RequestKeyFrame = true;
  m_UnPackParams.PackSeqNo = 0xFFFFFFFF;
}

int RtpDepacketizerVp8::VideoUnPackVp8RtpStm(
    const char* pData, int Length, std::vector<std::string>* o_h264Frames) {
  int ret = 0;

  if (!pData || Length == 0) {
    tylog("data null");
    assert(0);  // tmp assert for debug
    return -1;
  }

  RtpHeader* pRtp = (RtpHeader*)(pData);
  unsigned short Seq = pRtp->getSeqNumber();
  unsigned char PaddingLen = 0;
  if (pRtp->hasPadding()) {
    // should check length
    PaddingLen = *(pData + Length - 1);
  }

  // FIXME: should check length
  // int extOffet = GetRtpExtOffset((char*)pData, Length);
  // if (extOffet < 0 || Length < sizeof(RtpHeader) + extOffet) {
  //   tylog("[GetRtpExtOffset] err %d,Length:%d", extOffet, Length);
  //   return -2;
  // }
  int HeadLen = pRtp->getHeaderLength();  //  sizeof(RtpHeader) + extOffet;
  int PayloadLen = Length - HeadLen - PaddingLen;
  if (PayloadLen <= 0) {
    tylog(
        "payloadLen=%d, may be probe packet (all padding), NOTE: update shit "
        "PackSeqNo",
        PayloadLen);

    m_UnPackParams.PackSeqNo = Seq;

    return 0;
  }

  InitRTPVideoHeaderVP8();
  const int DescriptorSize =
      ParseVP8Descriptor(&m_RtpHeadInfoVp8, pData + HeadLen, PayloadLen);
  if (DescriptorSize < 0) {
    tylog("parseVP8Descriptor ret=%d, HeadLen %d,PaddingLen %d,PayloadLen %d",
          DescriptorSize, HeadLen, PaddingLen, PayloadLen);
    assert(0);  // tmp assert for debug
    return -3;
  }

  if (m_RtpHeadInfoVp8.PartitionId > 8) {
    // Weak check for corrupt payload_data: PartID MUST NOT be larger than 8.
    tylog(
        "Weak check for corrupt payload_data: PartID=%d MUST NOT be larger "
        "than 8",
        m_RtpHeadInfoVp8.PartitionId);
    return -4;
  }

  bool IsFirstPacket = m_RtpHeadInfoVp8.BeginningOfPartition &&
                       m_RtpHeadInfoVp8.PartitionId == 0;
  int Vp8PayloadSize = PayloadLen - DescriptorSize;
  if (Vp8PayloadSize == 0) {
    tylog("Empty vp8 payload");
    return -5;
  }

  const char* Vp8Payload = pData + HeadLen + DescriptorSize;
  // Read P bit from payload header (only at beginning of first partition).
  if (IsFirstPacket && (*Vp8Payload & 0x01) == 0) {
    is_key_frame = true;
    if (Vp8PayloadSize < 10) {
      // For an I-frame we should always have the uncompressed VP8 header
      // in the beginning of the partition.
      tylog("vp8 payload size=%d < 10, err", Vp8PayloadSize);

      return -6;
    }
    Width_ = ((Vp8Payload[7] << 8) + Vp8Payload[6]) & 0x3FFF;
    Height_ = ((Vp8Payload[9] << 8) + Vp8Payload[8]) & 0x3FFF;
    tylog("first get width=%d, height=%d.", Width_, Height_);
  }

  if (m_UnPackParams.RequestKeyFrame) {
    if (!is_key_frame) {
      tylog("is_key_frame is not key frame");
      return -1;
    }
    m_UnPackParams.RequestKeyFrame = false;
    m_UnPackParams.PackSeqNo = 0xFFFFFFFF;
  }

  if (static_cast<uint32_t>(0xFFFFFFFF) != m_UnPackParams.PackSeqNo) {
    int LostPktCnt = ((Seq - m_UnPackParams.PackSeqNo - 1 + 0x10000) & 0xFFFF);

    if (0 < LostPktCnt) {
      tylog("lost %d packets, pre_seq_num(%u), CurSeqNum(%u)!", LostPktCnt,
            m_UnPackParams.PackSeqNo, Seq);

      m_UnPackParams.RequestKeyFrame = true;
      // TODO: req I frame

      // if (m_pFastUpDate) {
      //   m_pFastUpDate(m_pUserPtr,  m_DataType);
      // }
      m_RawDataLen = 0;
      is_key_frame = false;

      return -7;
    }
  }

  if (m_RawDataLen + Vp8PayloadSize <= VP8_RAW_DATA_LEN) {
    memcpy(m_Vp8RawData + m_RawDataLen, Vp8Payload, Vp8PayloadSize);
    m_RawDataLen += Vp8PayloadSize;
  } else {
    tylog("too big Vp8RawData len[%d],m_RawDataLen[%d], Vp8PayloadSize[%d]",
          m_RawDataLen + Vp8PayloadSize, m_RawDataLen, Vp8PayloadSize);

    // tmp
    assert(!"too big shit");
  }

  if (pRtp->getMarker()) {
    // end of frame, or should use ts change to check if new frame?
    // https://github.com/meetecho/janus-gateway/blob/master/src/postprocessing/pp-webm.c#L349
    if (decoder == NULL) {
      decoder = new CodecDecoder();
      CodecParam Param;

      tylog("before decode, width=%d, height=%d.", Width_, Height_);
      assert(Width_ != 0 && Height_ != 0);
      // for decode not very care
      Param.width = Width_;
      Param.height = Height_;

      Param.codecName = "libvpx";
      if (!decoder->InitDecoder(Param)) {
        tylog("InitDecoder failed");
      }
    }

    ret = this->belongingRtpHandler_.WriteWebmFile(
        {m_Vp8RawData, m_Vp8RawData + m_RawDataLen}, pRtp->getTimestamp(),
        kMediaTypeVideo, is_key_frame);
    if (ret) {
      tylog("write webm video file ret=%d.", ret);

      return ret;
    }

    AVFrame* yuvFrame = decoder->Decode((uint8_t*)m_Vp8RawData, m_RawDataLen);
    bool ChangeResolution = false;
    if (yuvFrame) {
      ChangeResolution = encoder_width != (uint32_t)yuvFrame->width ||
                         encoder_height != (uint32_t)yuvFrame->height;
      if (ChangeResolution) {
        if (0 == encoder_width || 0 == encoder_height) {
          tylog(
              "first set resolution, last width=%d, new width=%d, last "
              "height=%d, new height=%d",
              encoder_width, yuvFrame->width, encoder_height, yuvFrame->height);
        } else {
          tylog(
              "normal change resolution, last width=%d, new width=%d, last "
              "height=%d, new height=%d",
              encoder_width, yuvFrame->width, encoder_height, yuvFrame->height);
        }
      }
    }
    if ((encoder == NULL || ChangeResolution) && yuvFrame) {
      if (!encoder) {
        encoder = new CodecEncoder();
      }

      encoder_width = yuvFrame->width;
      encoder_height = yuvFrame->height;
      CodecParam Param;
      Param.width = yuvFrame->width;
      Param.height = yuvFrame->height;

      // to constant ?
      Param.bitRate = 700000;
      Param.codecName = "libx264";

      // 如果分辨率改变了要重新建立编码器
      if (!encoder->InitEncoder(Param)) {
        tylog("InitEncoder failed");

        return -1;
      }
    }
    AVPacket* h264Packet = NULL;
    assert(nullptr != encoder);  // tmp, not to use ptr
    if (encoder) {
      h264Packet = encoder->Encode(yuvFrame, is_key_frame, true);
    }
    if (h264Packet) {
      tylog("avpacket size=%d, dts=%lu, pts=%lu, nowMs=%lu.", h264Packet->size,
            h264Packet->dts, h264Packet->pts, g_now_ms);
      o_h264Frames->emplace_back(h264Packet->data,
                                 h264Packet->data + h264Packet->size);
    }
    encoder->UnrefPacket();

    m_RawDataLen = 0;
    is_key_frame = false;
  }

  m_UnPackParams.PackSeqNo = Seq;

  return 0;
}

}  // namespace tywebrtc
