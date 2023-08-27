#include "rtmp/rtmp_handler.h"

#include <poll.h>

#include <cassert>

#include "colib/co_routine.h"
#include "tylib/time/timer.h"

#include "global_tmp/global_tmp.h"
#include "log/log.h"

RtmpHandler::RtmpHandler(PeerConnection &pc)
    : belongingPeerConnection_(pc), flvAssist(*this) {}

RtmpHandler::~RtmpHandler() {
  if (mRtmpInstance != NULL) {
    RTMP_Close(mRtmpInstance);
    RTMP_Free(mRtmpInstance);
  }
}

// init rtmp and connect
// OPT: block mode
int RtmpHandler::InitProtocolHandler(const std::string &rtmpUrl) {
  int ret = 0;

  ret = InitRtmp_();
  if (ret) {
    return ret;
  }

  ret = SetupRtmp_(rtmpUrl, true, 1000, true);
  if (ret) {
    return ret;
  }

  ret = ConnectRtmp_();
  if (ret) {
    return ret;
  }

  return 0;
}

bool RtmpHandler::InitSucc() const { return initSucc_; }

// to cancle wrap
int RtmpHandler::SendAudioFrame(const std::vector<char> &audioFrame,
                                uint64_t frameMs) {
  return flvAssist.SendAudioFrame(audioFrame, frameMs);
}

int RtmpHandler::SendVideoFrame(const std::vector<char> &h264Frame,
                                uint64_t frameMs) {
  return flvAssist.SendVideoFrame(h264Frame, frameMs);
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

// no network operation
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

// no network operation
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

  if (!RTMP_SetupURL(mRtmpInstance, const_cast<char *>(mRtmpUrl.c_str()))) {
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
  assert(RTMP_IsConnected(mRtmpInstance));

  uint64_t timeEnd = g_now_ms;  // same as timeBegin?
  uint64_t costTime = timeEnd - timeBegin;
  tylog("RTMP_Connect success, timecostMs=%lu, fd=%d.", costTime,
        mRtmpInstance->m_sb.sb_socket);
  if (1000 < costTime) {
    tylog("RTMP_Connect cost too long, timecost:%lu (ms)", costTime);
  }

  // OPT: should set in RTMP_Connect
  // ret = SetNonBlock(mRtmpInstance->m_sb.sb_socket);
  // if (ret) {
  //   tylog("setNonBlock ret=%d.", ret);

  //   return ret;
  // }

  timeBegin = g_now_ms;

  // struct pollfd pf = {0, 0, 0};
  // pf.fd = mRtmpInstance->m_sb.sb_socket;
  // pf.events = (POLLIN | POLLERR | POLLHUP | POLLOUT);
  // co_poll(co_get_epoll_ct(), &pf, 1, 10000);

  // may stop when ConnectStream
  if (!RTMP_IsConnected(mRtmpInstance)) {
    tylog("rtmp is not connected, not connectStream, return");
    // TODO: how to free self?
    return -24;
  }

  ok = RTMP_ConnectStream(mRtmpInstance, 0);
  if (!ok) {
    if (errno == EAGAIN) {
      tylog("rtmp connectStream not ok (block mode)");

      return -233;
    } else {
      tylog("connectStream err, stop connect, errno=%d[%s], rtmp=%s.", errno,
            strerror(errno), RTMPToString(*mRtmpInstance).data());
      // should close rtmp?
      return -928;
    }
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

  initSucc_ = true;

  return 0;
}

// now no use
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
