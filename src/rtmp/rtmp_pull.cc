// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtmp/rtmp_pull.h"

#include "colib/co_routine.h"

#include "src/codec/audio_codec.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

void* MyRtmpHandle(void* pRtmpAssist) {
  tylog("enter myRtmpHandle");
  co_enable_hook_sys();

  const int Idx = 0;

  RtmpAssist& rtmpPuller = *reinterpret_cast<RtmpAssist*>(pRtmpAssist);
  // now pull only one stream, use first position
  RTMP* pRtmp = &rtmpPuller.m_Clients[0].rtmp;

  for (;;) {
    struct pollfd pf = {0, 0, 0};
    pf.fd = pRtmp->m_sb.sb_socket;
    pf.events = (POLLIN | POLLERR | POLLHUP);
    co_poll(co_get_epoll_ct(), &pf, 1, 10000);

    // may stop when ConnectStream
    if (!RTMP_IsConnected(pRtmp)) {
      tylog("rtmp is not connected, not connectStream, return");
      // TODO: how to free self?
      return nullptr;
    }

    bool ok = RTMP_ConnectStream(pRtmp, 0);
    if (!ok) {
      if (errno == EAGAIN) {
        tylog("rtmp connectStream not ok, but not error (nonblock mode)");

        continue;
      } else {
        tylog("connectStream err, stop connect, errno=%d[%s], rtmp=%s.", errno,
              strerror(errno), RTMPToString(*pRtmp).data());
        // should close rtmp?
        return nullptr;
      }
    } else {
      break;
    }
  }

  tylog("socketfd:%d, Connect Server all ok tcURL=%s.", pRtmp->m_sb.sb_socket,
        pRtmp->Link.tcUrl.av_val);

  assert(pRtmp->m_sb.sb_socket > 0);

  rtmpPuller.m_Clients[Idx].State = RTMP_STATE_PLAY;

  rtmpPuller.m_Clients[Idx].acceptFd = pRtmp->m_sb.sb_socket;

  struct stat tStat;
  if (-1 == fstat(pRtmp->m_sb.sb_socket, &tStat)) {
    tylog("fstat fd:%d error=%d[%s]", pRtmp->m_sb.sb_socket, errno,
          strerror(errno));
    // to destruct client
    // may should not use assert
    assert(!"shit rtmp fd");
  }

  assert(RTMP_IsConnected(pRtmp));

  int ret = rtmpPuller.HandleRtmpFd(pRtmp->m_sb.sb_socket);
  if (ret) {
    tylog("handleRequest fail, ret=%d, to close rtmp", ret);
    RTMP_Close(pRtmp);
    tylog("close rtmp done");

    return nullptr;
  }

  return nullptr;
}

RtmpPuller::RtmpPuller(PeerConnection& pc) : belongingPC_(pc) {}

int RtmpPuller::InitProtocolHandler(const std::string& Url) {
  int ret = 0;

  tylog("rtmp url:%s.", Url.c_str());

  // now pull only one stream, use first position
  RTMP* pRtmp = &m_Clients[0].rtmp;
  RTMP_Init(pRtmp);

  const int Idx = 0;
  SetupRtmp(-1, Idx);

  m_Clients[Idx].StartConnectTime = g_now_ms;
  m_Clients[Idx].ClientIp = (unsigned int)0;

  // should set nonblock
  bool ok = RTMP_SetupURL(pRtmp, (char*)Url.c_str());
  if (!ok) {
    tylog("rtmp setup url not ok");
    return -1;
  }
  tylog("rtmp setup URL ok");

  if (pRtmp->Link.lFlags & RTMP_LF_FTCU) {
    free(pRtmp->Link.tcUrl.av_val);
    pRtmp->Link.tcUrl.av_val = NULL;
    pRtmp->Link.lFlags ^= RTMP_LF_FTCU;
  }

  // pRtmp->Link.tcUrl.av_val = (char*)Tcurl.c_str();
  // pRtmp->Link.tcUrl.av_len = Tcurl.length();

  // 设置直播标志
  pRtmp->Link.lFlags |= RTMP_LF_LIVE;

  RTMP_SetBufferMS(pRtmp, 4 * 3600 * 1000);

  // RTMP_EnableWrite(rtmp); // KEY: means push stream, not play!

  // create socket(fd)
  // OPT: use non-block connect
  // int ret = RTMP_Connect_NonBlock(pRtmp);
  ok = RTMP_Connect(pRtmp, nullptr);
  if (!ok) {
    tylog("rtmp connect not ok");
    return -1;
  }

  assert(RTMP_IsConnected(pRtmp));
  tylog("rtmp fd=%d connect ok", pRtmp->m_sb.sb_socket);

  // OPT: should set in RTMP_Connect
  // ret = SetNonBlock(pRtmp->m_sb.sb_socket);
  // if (ret) {
  //   tylog("setNonBlock ret=%d.", ret);

  //   return ret;
  // }

  stCoRoutine_t* co = nullptr;
  ret = co_create(&co, nullptr, MyRtmpHandle, this);
  if (ret) {
    tylog("co create ret=%d.", ret);

    return ret;
  }
  co_resume(co);

  tylog("co_resume done, this=%p.", this);

  // co_release(co); ?

  return 0;
}

}  // namespace tywebrtc