#ifndef RTMP_RTMP_HANDLER_H_
#define RTMP_RTMP_HANDLER_H_

#include <cstdint>
#include <string>
#include <vector>

#include "librtmp/rtmp.h"

#include "rtmp/FlvAssist.h"

// #include "user_basic_info_struct.h"

const uint32_t DEFAULT_TIMEOUT = 30;  // 30 seconds

class RtmpHandler {
 public:
  RtmpHandler();

  // note assignment and copy constructor
  ~RtmpHandler();

 public:
  int InitRtmpConnection(std::string rtmpUrl, bool writeable, uint32_t timeout,
                         const bool liveSource);

  int SendVideoFrame(const std::vector<char> &h264Frame, time_t frameTs);
  int SendAudioFrame(const std::string &audioFrame, time_t frameTs);

  int RecvRtmpPacket(std::vector<char> *o_recvBuf);

  // tmp public, indicate if rtmp setup is ok (not null)
  RTMP *mRtmpInstance = nullptr;

 private:
  int InitRtmp_();
  int SetupRtmp_(std::string rtmpUrl, bool writeable, uint32_t timeout,
                 bool liveSource);
  int ConnectRtmp_();

  int ReconnectRtmp_();

 private:
  FlvAssist flvAssist;
  uint32_t mMaxRtmpPacketLength = 0;

  std::string mRtmpUrl;
  bool mWriteable = false;
  uint32_t mTimeOut = DEFAULT_TIMEOUT;
  bool mLiveSource = false;

  // RTMPPacket* mRtmpPacket = nullptr;
};

#endif  // RTMP_RTMP_HANDLER_H_
