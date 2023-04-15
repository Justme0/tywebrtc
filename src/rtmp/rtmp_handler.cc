#include "rtmp/rtmp_handler.h"

#include "log/log.h"
#include "tylib/time/timer.h"

RtmpHandler::RtmpHandler() : flvAssist(*this) {}

RtmpHandler::~RtmpHandler() {
  if (mRtmpInstance != NULL) {
    RTMP_Close(mRtmpInstance);
    RTMP_Free(mRtmpInstance);
  }
}

// init rtmp and connect
// OPT: block mode
int RtmpHandler::InitProtocolHandler(const std::string &rtmpUrl, bool writeable,
                                     uint32_t timeout, const bool liveSource) {
  int ret = 0;

  ret = InitRtmp_();
  if (ret) {
    return ret;
  }

  ret = SetupRtmp_(rtmpUrl, writeable, timeout, liveSource);
  if (ret) {
    return ret;
  }

  ret = ConnectRtmp_();
  if (ret) {
    return ret;
  }

  return 0;
}

bool RtmpHandler::InitSucc() const { return nullptr != this->mRtmpInstance; }

// to cancle repeat
int RtmpHandler::SendAudioFrame(const std::vector<char> &audioFrame,
                                uint64_t frameMs) {
  return flvAssist.SendAudioFrame(audioFrame, frameMs);
}

int RtmpHandler::SendVideoFrame(const std::vector<char> &h264Frame,
                                uint64_t frameMs) {
  return flvAssist.SendVideoFrame(h264Frame, frameMs);

  //   if (!RTMP_IsConnected(
  //           mRtmpInstance) /*&& reconnect(push_key, outip, selfPort) != 0*/)
  //           {
  //     return -2;
  //   }

  /*
    if (NULL == flvTag || flvTagLength < 11) {
      return -3;
    }

  #if 0
          if (flvTagLength > mMaxRtmpPacketLength)
          {
                  RTMPPacket_Free(mRtmpPacket);
                  mMaxRtmpPacketLength = flvTagLength * 2;
                  RTMPPacket_Alloc(mRtmpPacket, mMaxRtmpPacketLength);
                  RTMPPacket_Reset(mRtmpPacket);
          }

          unsigned char tmp[12] = { 0 };
          memcpy(tmp, flvTag, 11);

          mRtmpPacket->m_headerType              = RTMP_PACKET_SIZE_LARGE;
          mRtmpPacket->m_packetType               = flvTag[0];
          mRtmpPacket->m_nChannel                   = 0x04;
          mRtmpPacket->m_nBodySize                 = 0 | flvTag[1] << 16 |
  flvTag[2] << 8 | flvTag[3];
          if (mRtmpPacket->m_nBodySize > flvTagLength - 11)
          {
                  return -4;
          }
          mRtmpPacket->m_nTimeStamp            = 0 |  flvTag[4] << 16 |
  flvTag[5] << 8 | flvTag[6];
          mRtmpPacket->m_hasAbsTimestamp = flvTag[7];
          mRtmpPacket->m_nInfoField2               = mRtmpInstance->m_stream_id;
          memcpy(mRtmpPacket->m_body, flvTag + 11, mRtmpPacket->m_nBodySize);

          int ret = RTMP_SendPacket(mRtmpInstance, mRtmpPacket, 0);
  #else
    int ret = RTMP_Write(mRtmpInstance, (char *)flvTag, flvTagLength);
  #endif

    tylog("Try to send rtmp pkg(Len=%u), Ret=%d", flvTagLength, ret);

    return ret < 1 ? -5 : 0;
    */
}

// now no use
int RtmpHandler::RecvRtmpPacket(std::vector<char> *o_recvBuf) {
  if (NULL == mRtmpInstance) {
    return -1;
  }

  if (!RTMP_IsConnected(mRtmpInstance) && ReconnectRtmp_() != 0) {
    return -2;
  }

  o_recvBuf->resize(4096);

  int ret = RTMP_Read(mRtmpInstance, o_recvBuf->data(), o_recvBuf->size());
  if (ret < 0) {
    tylog("err: read rtmp pkg ret=%d.", ret);
    return ret;
  }

  tylog("ok: read rtmp pkg size=%d.", ret);

  o_recvBuf->resize(ret);

  return 0;
}

int RtmpHandler::InitRtmp_() {
  if (mRtmpInstance != NULL) {
    return 0;
  }

  mRtmpInstance = RTMP_Alloc();
  if (NULL == mRtmpInstance) {
    return -1;
  }

  RTMP_Init(mRtmpInstance);

  return 0;
}

int RtmpHandler::SetupRtmp_(std::string rtmpUrl, bool writeable,
                            uint32_t timeout, const bool liveSource) {
  if (NULL == mRtmpInstance) {
    tylog("rtmp connect setup failed\n");

    return -1;
  }

  if (rtmpUrl.empty()) {
    tylog("rtmp connect setup failed, rtmpurl_size=%zu mRtmpUrl.size=%zu",
          rtmpUrl.size(), mRtmpUrl.size());
    RTMP_Free(mRtmpInstance);
    mRtmpInstance = NULL;

    return -2;
  }

  mRtmpUrl = rtmpUrl;
  mWriteable = writeable;
  // TODO: timeout和livesource单元测试
  mTimeOut = mRtmpInstance->Link.timeout = timeout;
  if (liveSource) {
    mRtmpInstance->Link.lFlags |= RTMP_LF_LIVE;
  }
  mLiveSource = liveSource;

  if (!RTMP_SetupURL(mRtmpInstance, (char *)mRtmpUrl.c_str())) {
    RTMP_Free(mRtmpInstance);
    mRtmpInstance = NULL;
    mRtmpUrl.clear();

    return -3;
  }

  if (mWriteable) {
    RTMP_EnableWrite(mRtmpInstance);
  }

  RTMP_SetBufferMS(mRtmpInstance, 4 * 3600 * 1000);  // 4 hours

  return 0;
}

/*
int RTMP_SwitchToNonBlocking(RTMP *r)
{
    //http mode do not open the non-blocking.
        if (r->m_sb.sb_socket != -1 && !(r->Link.protocol & RTMP_FEATURE_HTTP)){
                int val = 0;
            int fd_ = r->m_sb.sb_socket;

            if ((val = fcntl(fd_,F_GETFL,0)) < 0)
            {
                return -1;
            }

            val |= O_NONBLOCK;
            if (fcntl(fd_,F_SETFL,val) == -1)
            {
                return -1;
            }
                r->m_switchToNonBlock = 1;
                return 1;
        }
        return -1;
}
*/

// return error code
int RtmpHandler::ConnectRtmp_() {
  if (NULL == mRtmpInstance) {
    return -1;
  }

  uint64_t timeBegin = g_now_ms;
  // int ret = RTMP_Connect(mRtmpInstance, NULL, outip, selfPort, &dnsresutl);
  bool ok = RTMP_Connect(mRtmpInstance, NULL);
  if (!ok) {
    tylog("RTMP_Connect fail");
    RTMP_Free(mRtmpInstance);
    mRtmpInstance = NULL;
    mRtmpUrl.clear();
    return -2;
  }

  uint64_t timeEnd = g_now_ms;  // same as timeBegin?
  uint64_t costTime = timeEnd - timeBegin;
  tylog("RTMP_Connect success, timecost:%lu (ms)", costTime);
  if (1000 < costTime) {
    tylog("RTMP_Connect cost too long, timecost:%lu (ms)", costTime);
  }

  timeBegin = g_now_ms;
  if (!RTMP_ConnectStream(mRtmpInstance, 0)) {
    tylog("RTMP_ConnectStream fail");
    RTMP_Close(mRtmpInstance);
    RTMP_Free(mRtmpInstance);
    mRtmpInstance = NULL;
    mRtmpUrl.clear();
    return -3;
  }

  // if (RTMP_SwitchToNonBlocking(mRtmpInstance)) {
  //   tylog("RTMP_ConnectStream, success to set the connection to
  //   non-blocking");
  // }

  timeEnd = g_now_ms;  // same as timeBegin?
  costTime = timeEnd - timeBegin;
  tylog("RTMP_ConnectStream success, timecost:%lu (ms)", costTime);
  if (1000 < costTime) {
    tylog("connect cost too long=%ld.", costTime);
  }

  return 0;
}

int RtmpHandler::ReconnectRtmp_() {
  int ret = SetupRtmp_(mRtmpUrl, mWriteable, mTimeOut, mLiveSource);
  if (ret != 0) {
    tylog("rtmp reconnect failed, ret=%d\n", ret);
    return -1;
  }

  tylog("in rtmp reconnect\n");
  if (ConnectRtmp_() != 0) {
    return -2;
  }

  tylog("set mVideoConfigTag in reconnect");

  return 0;
}
