// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/global_tmp/global_tmp.h"

#include "colib/co_routine.h"
#include "tylib/ip/ip.h"
#include "tylib/time/timer.h"

#include "src/log/log.h"
#include "src/pc/peer_connection.h"

namespace tywebrtc {

prometheus::Family<prometheus::Gauge> *g_startServer;
prometheus::Family<prometheus::Gauge> *g_recvPacketNum;

int g_sock_fd;
int g_dumpRecvSockfd;
int g_dumpSendSockfd;

// 收到 rtp/rtcp 时dump，注意下行也会收到rtcp
void DumpRecvPacket(const std::vector<char> &packet) {
  sockaddr_in addr = tylib::ConstructSockAddr("127.0.0.1", 12347);
  ssize_t sendtoLen =
      sendto(g_dumpRecvSockfd, packet.data(), packet.size(), 0,
             reinterpret_cast<sockaddr *>(&addr), sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    // return -1;
  }
}

// 发包前dump未加密的包，包括发给源client和下游peer
void DumpSendPacket(const std::vector<char> &packet) {
  sockaddr_in addr = tylib::ConstructSockAddr("127.0.0.1", 12347);
  ssize_t sendtoLen =
      sendto(g_dumpSendSockfd, packet.data(), packet.size(), 0,
             reinterpret_cast<sockaddr *>(&addr), sizeof(struct sockaddr_in));
  if (-1 == sendtoLen) {
    tylog("sendto errorno=%d[%s]", errno, strerror(errno));
    // return -1;
  }
}

// from https://ffmpeg.org/doxygen/3.0/log_8c_source.html#l00224
// origin is `static` so we copy it
static const char *my_get_level_str(int level) {
  switch (level) {
    case AV_LOG_QUIET:
      return "quiet";
    case AV_LOG_PANIC:
      return "panic";
    case AV_LOG_FATAL:
      return "fatal";
    case AV_LOG_ERROR:
      return "error";
    case AV_LOG_WARNING:
      return "warning";
    case AV_LOG_INFO:
      return "info";
    case AV_LOG_VERBOSE:
      return "verbose";
    case AV_LOG_DEBUG:
      return "debug";
    case AV_LOG_TRACE:
      return "trace";
    default:
      return "";
  }
}

// from rtmp lib static var
static const char *levels[] = {"CRIT", "ERROR", "WARNING",
                               "INFO", "DEBUG", "DEBUG2"};

class SrsFFmpegLogHelper {
 public:
  SrsFFmpegLogHelper() {
    // ffmpeg
    av_log_set_level(AV_LOG_ERROR);
    av_log_set_callback(ffmpegLog);

    // rtmp lib
    RTMP_LogSetLevel(RTMP_LOGALL);
    RTMP_LogSetCallback(rtmpLog);

    // libco
    libco_log_set_callback(libcoLog);
  }

  // from librtmp/log.c rtmp_log_default()
  static void rtmpLog(int level, const char *format, va_list vl) {
    if (level > RTMP_debuglevel) {
      return;
    }

#define MAX_PRINT_LEN 2048
    char str[MAX_PRINT_LEN] = "";

    vsnprintf(str, MAX_PRINT_LEN - 1, format, vl);

    // should check level value
    tylog("%s %s", levels[level], str);
  }

  // @brief FFmpeg log callback function
  // reference: void format_line(void *avcl, int level, const char *fmt, va_list
  // vl, AVBPrint part[4],
  // int *print_prefix, int type[2]) at
  // https://ffmpeg.org/doxygen/3.4/log_8c_source.html#l00248
  static void ffmpegLog(void *avcl, int level, const char *fmt, va_list vargs) {
    // if (level > AV_LOG_TRACE) {
    //   return;
    // }

    std::stringstream nameInfo;

    AVClass *avc = avcl ? *(AVClass **)avcl : NULL;
    if (avc) {
      if (avc->parent_log_context_offset) {
        AVClass **parent =
            *(AVClass ***)(((uint8_t *)avcl) + avc->parent_log_context_offset);
        if (parent && *parent) {
          nameInfo << "parent [" << (*parent)->item_name(parent) << " @ "
                   << parent << "] ";
        }
      }
      nameInfo << "[" << avc->item_name(avcl) << " @ " << avcl << "]";
    }

    static char codecInfo[4096] = {0};
    int nbytes = vsnprintf(codecInfo, sizeof(codecInfo), fmt, vargs);
    if (nbytes > 0 && nbytes < (int)sizeof(codecInfo)) {
      // Srs log is always start with new line, replcae '\n' to '\0', make log
      // easy to read.
      if (codecInfo[nbytes - 1] == '\n') {
        codecInfo[nbytes - 1] = '\0';
      }

      tylog("%s %s %s", my_get_level_str(level), nameInfo.str().data(),
            codecInfo);
    }
  }

  static void libcoLog(const char *format, va_list args) {
    tylog(format, args);
  }
};

// Register FFmpeg log callback funciton.
SrsFFmpegLogHelper _srs_ffmpeg_log_helper;
}  // namespace tywebrtc
