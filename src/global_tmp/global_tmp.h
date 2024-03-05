// OPT: not use global thing as possible
// will refactor

#pragma once

#include <fcntl.h>
#include <sys/stat.h>

#include <cassert>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "librtmp/rtmp.h"
#include "prometheus/family.h"
#include "prometheus/gauge.h"
#include "tylib/string/format_string.h"

#include "log/log.h"

const int kUplossRateMul100 = 0;
const int kDownlossRateMul100 = 0;

// should be from BWE
const double kFecRate = 0.1;

const int kPCDeadTimeoutMs = 3 * 1000;

// https://qr.ae/pK6JH5
// https://qr.ae/pK6Jkf
// MTU by itself is meaningless, but usually means the L2 (link layer) MTU, the
// total size of a frame that can be sent on a specific networking medium,
// excluding the preamble and FCS (CRC).
const int kGuessMtuByte = 1200;

extern prometheus::Family<prometheus::Gauge>* g_startServer;
extern prometheus::Family<prometheus::Gauge>* g_recvPacketNum;

extern int g_sock_fd;
extern int g_dumpRecvSockfd;
extern int g_dumpSendSockfd;

const int kDownlinkAudioSsrc = 16854838;  // taylor to make dynamic
const int kDownlinkAudioPayloadType = 111;

const int kDownlinkVideoSsrc = 33697348;  // taylor to make dynamic
const int kDownlinkH264PayloadType = 106;

const int kDownlinkVideoFecSsrc = 13697341;  // taylor to make dynamic

class PeerConnection;

// TODO: should save to remote DB ? must refactor! Now we use singleton
// OPT2: move to tylib
template <class T>
class Singleton : private T {
 public:
  // may have param
  static T& Instance() {
    // 1. C++11: If control enters the declaration concurrently while the
    // variable is being initialized, the concurrent execution shall wait for
    // completion of the initialization.
    // 2. Lazy evaluation.
    static Singleton<T> s;

    return s;
  }

  Singleton(const Singleton&) = delete;
  Singleton& operator=(const Singleton&) = delete;

 private:
  Singleton() {}
  ~Singleton() {}
};

// manage all peerConnection, in singleton
class PCManager {
 public:
  struct ClientSrcId {
    std::string ip;
    int port;
    ClientSrcId(const std::string& ip, int port) : ip(ip), port(port) {}

    bool operator<(const ClientSrcId& that) const {
      return std::tie(ip, port) < std::tie(that.ip, that.port);
    }
  };

  int GetPeerConnectionSize() const { return client2PC_.size(); }

  void CleanTimeoutPeerConnection();

  std::shared_ptr<PeerConnection> GetPeerConnection(const std::string& ip,
                                                    int port,
                                                    const std::string& ufrag);

  std::shared_ptr<PeerConnection> GetPeerConnection(int targetFd) const;

  // should be private
 public:
  // don't use global variable STL
  // taylor 定时清理超时会话（pc）
  // std::map is not designed to work with objects which are not
  // copy-constructible. But PeerConnection cannot be copy because its member
  // has reference data member
  // https://stackoverflow.com/questions/20972751/how-to-put-a-class-that-has-deleted-copy-ctor-and-assignment-operator-in-map
  std::map<ClientSrcId, std::shared_ptr<PeerConnection>> client2PC_;
};

void DumpRecvPacket(const std::vector<char>& packet);
void DumpSendPacket(const std::vector<char>& packet);

// tmp
inline int mkdir_p(const char* path, mode_t mode) {
  const char* p;
  p = strchr(path + 1, '/');

  struct stat st;
  while (1) {
    if (!p) {
      int n;
      if ((n = strlen(path)) > 0 && path[n - 1] != '/') {
        if (stat(path, &st) < 0 && errno == ENOENT &&
            (mkdir(path, mode) < 0 || chmod(path, mode) < 0))
          return -1;
      }
      break;
    }

    std::string name = std::string(path, p - path);

    if (stat(name.c_str(), &st) < 0 && errno == ENOENT &&
        (mkdir(name.c_str(), mode) < 0 || chmod(name.c_str(), mode) < 0))
      return -2;

    p = strchr(p + 1, '/');
  }

  return 0;
}

inline int SetNonBlock(int iSock) {
  int iFlags = fcntl(iSock, F_GETFL, 0);
  if (iFlags == -1) {
    tylog("fcntl return -1, errno=%d [%s]", errno, strerror(errno));

    return -1;
  }
  iFlags |= O_NONBLOCK;
  iFlags |= O_NDELAY;
  int ret = fcntl(iSock, F_SETFL, iFlags);
  if (ret == -1) {
    tylog("fcntl return -1, errno=%d [%s]", errno, strerror(errno));

    return -2;
  }

  return 0;
}

inline std::string RtmpHeaderTypeToString(int headerType) {
  switch (headerType) {
    case RTMP_PACKET_SIZE_LARGE:
      return "LARGE";
    case RTMP_PACKET_SIZE_MEDIUM:
      return "MEDIUM";
    case RTMP_PACKET_SIZE_SMALL:
      return "SMALL";
    case RTMP_PACKET_SIZE_MINIMUM:
      return "MINIMUM";

    default:
      return "unknownHeaderType[" + std::to_string(headerType) + "]";
  }
}

inline std::string RtmpPacketTypeToString(int rtmpPacketType) {
  switch (rtmpPacketType) {
    case RTMP_PACKET_TYPE_CHUNK_SIZE:
      return "RTMP_PACKET_TYPE_CHUNK_SIZE";
    case RTMP_PACKET_TYPE_BYTES_READ_REPORT:
      return "RTMP_PACKET_TYPE_BYTES_READ_REPORT";
    case RTMP_PACKET_TYPE_CONTROL:
      return "RTMP_PACKET_TYPE_CONTROL";
    case RTMP_PACKET_TYPE_SERVER_BW:
      return "RTMP_PACKET_TYPE_SERVER_BW";
    case RTMP_PACKET_TYPE_CLIENT_BW:
      return "RTMP_PACKET_TYPE_CLIENT_BW";
    case RTMP_PACKET_TYPE_AUDIO:
      return "RTMP_PACKET_TYPE_AUDIO";
    case RTMP_PACKET_TYPE_VIDEO:
      return "RTMP_PACKET_TYPE_VIDEO";
    case RTMP_PACKET_TYPE_FLEX_STREAM_SEND:
      return "RTMP_PACKET_TYPE_FLEX_STREAM_SEND";
    case RTMP_PACKET_TYPE_FLEX_SHARED_OBJECT:
      return "RTMP_PACKET_TYPE_FLEX_SHARED_OBJECT";
    case RTMP_PACKET_TYPE_FLEX_MESSAGE:
      return "RTMP_PACKET_TYPE_FLEX_MESSAGE";
    case RTMP_PACKET_TYPE_INFO:
      return "RTMP_PACKET_TYPE_INFO";
    case RTMP_PACKET_TYPE_SHARED_OBJECT:
      return "RTMP_PACKET_TYPE_SHARED_OBJECT";
    case RTMP_PACKET_TYPE_INVOKE:
      return "RTMP_PACKET_TYPE_INVOKE";
    case RTMP_PACKET_TYPE_FLASH_VIDEO:
      return "RTMP_PACKET_TYPE_FLASH_VIDEO";
    default:
      return "unknownRtmpPacketType[" + std::to_string(rtmpPacketType) + "]";
  }
}

inline std::string RTMPChunkToString(const RTMPChunk& chunk) {
  // print head?
  return tylib::format_string("{headerSize=%d, chunkSize=%d, chunk=%p}",
                              chunk.c_headerSize, chunk.c_chunkSize,
                              chunk.c_chunk);
}

inline std::string RTMPPacketToString(const RTMPPacket& pkg) {
  return tylib::format_string(
      "{headerType=%s, packetType=%s, hasAbsTimestamp=%d, channel=%d, "
      "timeStamp=%u, infoField2=%d, bodySize=%u, bytesRead=%u, chunk=%s, "
      "body=%p}",
      RtmpHeaderTypeToString(pkg.m_headerType).data(),
      RtmpPacketTypeToString(pkg.m_packetType).data(), pkg.m_hasAbsTimestamp,
      pkg.m_nChannel, pkg.m_nTimeStamp, pkg.m_nInfoField2, pkg.m_nBodySize,
      pkg.m_nBytesRead,
      nullptr != pkg.m_chunk ? RTMPChunkToString(*pkg.m_chunk).data() : "null",
      pkg.m_body);
}

inline std::string RTMPSockBufToString(const RTMPSockBuf& b) {
  return tylib::format_string(
      "{sb_socket=%d, sb_size(unprocessedBytes)=%d, toProcessIndex=%s, "
      "timeout=%d, ssl=%p}",
      b.sb_socket, b.sb_size,
      b.sb_size == 0 ? "null" : std::to_string(b.sb_start - b.sb_buf).data(),
      b.sb_timedout, b.sb_ssl);
}

inline std::string AValToString(const AVal& name) {
  if (name.av_val == nullptr) {
    return "null";
  } else {
    return tylib::format_string("%.*s", name.av_len, name.av_val);
  }
}

inline std::string RtmpMethodToString(const RTMP_METHOD& method) {
  return tylib::format_string("{val=%s, num=%d}",
                              AValToString(method.name).data(), method.num);
}

inline std::string RTMPToString(const RTMP& r) {
  return tylib::format_string(
      "{m_inChunkSize=%d, m_outChunkSize=%d, m_nBWCheckCounter=%d, "
      "m_nBytesIn=%d, m_nBytesInSent=%d, m_nBufferMS=%d, m_stream_id=%d, "
      "m_mediaChannel=%d, m_mediaStamp=%u, m_pauseStamp=%u, m_pausing=%d, "
      "m_nServerBW=%d, m_nClientBW=%d, m_nClientBW2=%d, m_bPlaying=%d, "
      "m_bSendEncoding=%d, m_bSendCounter=%d, m_numInvokes=%d, m_numCalls=%d, "
      "m_methodCalls=%s, m_channelsAllocatedIn=%d, m_channelsAllocatedOut=%d, "
      "m_fAudioCodecs=%f, m_fVideoCodecs=%f, m_fEncoding=%f, m_fDuration=%f, "
      "m_msgCounter=%d, m_polling=%d, m_resplen=%d, m_unackd=%d, "
      "m_clientID=%s, m_write=%s, m_sb=%s}",

      r.m_inChunkSize, r.m_outChunkSize, r.m_nBWCheckCounter, r.m_nBytesIn,
      r.m_nBytesInSent, r.m_nBufferMS, r.m_stream_id, r.m_mediaChannel,
      r.m_mediaStamp, r.m_pauseStamp, r.m_pausing, r.m_nServerBW, r.m_nClientBW,
      r.m_nClientBW2, r.m_bPlaying, r.m_bSendEncoding, r.m_bSendCounter,
      r.m_numInvokes, r.m_numCalls,
      nullptr == r.m_methodCalls ? "null"
                                 : RtmpMethodToString(*r.m_methodCalls).data(),
      r.m_channelsAllocatedIn, r.m_channelsAllocatedOut, r.m_fAudioCodecs,
      r.m_fVideoCodecs, r.m_fEncoding, r.m_fDuration, r.m_msgCounter,
      r.m_polling, r.m_resplen, r.m_unackd, AValToString(r.m_clientID).data(),
      RTMPPacketToString(r.m_write).data(), RTMPSockBufToString(r.m_sb).data());
}

// C++ compile av_err2str err
// https://ffmpeg.org/pipermail/libav-user/2013-January/003458.html
#define av_err2string(errnum)                                         \
  av_make_error_string(                                               \
      static_cast<char*>(__builtin_alloca(AV_ERROR_MAX_STRING_SIZE)), \
      AV_ERROR_MAX_STRING_SIZE, errnum)
