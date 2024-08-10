// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/rtmp/rtmp_server.h"
#include "src/pc/peer_connection_manager.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace tywebrtc {

PeerConnection* RtmpServer::GetRtmpPeerPC() {
  return Singleton<PCManager>::Instance().FindOnePeerPC();
}

// @brief 从m_Socks中分配空闲位置保存fd，生成一个RTMP client
// 结构插入ClientsHead链表
// @param fd [in] accept(2) 返回的fd
// @return 达到TCP连接最大数返-1；否则返0
int RtmpServer::SetupClient(int fd) {
  // int sockflags = fcntl(fd, F_GETFL, 0);
  struct sockaddr_in DstAddr;
  socklen_t destlen = sizeof(DstAddr);
  getsockname(fd, (struct sockaddr*)&DstAddr, &destlen);
  // why DstAddr.sin_addr is also source IP ?

  struct sockaddr_in SrcAddr;
  socklen_t len = sizeof(SrcAddr);
  getpeername(fd, (struct sockaddr*)&SrcAddr, &len);

  int i = 0;
  // tmp
  // for (; i < MAXC; i++) {
  //   if (m_Clients[i].acceptFd == 0) {
  //     break;
  //   }
  // }

  if (MAXC == i) {
    tylog("No more client slots; increase?\n");

    return -1;
  }

  // fcntl(fd, F_SETFL, sockflags | O_NONBLOCK);
  m_Clients[i].acceptFd = fd;  // i为首个空闲的 fd 位置，分配给accept的fd
  g_fd2ClientIndex[fd] = i;  // FIXME: when remove it

  SetupRtmp(fd, i);
  m_Clients[i].ClientIp = (unsigned int)SrcAddr.sin_addr.s_addr;

  tylog("accepted connection from [%s:%u] at index %d, local port=%u",
        inet_ntoa(SrcAddr.sin_addr), ntohs(SrcAddr.sin_port), i,
        ntohs(DstAddr.sin_port));

  return 0;
}

}  // namespace tywebrtc