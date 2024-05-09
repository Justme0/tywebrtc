#ifndef RTMP_RTMP_HANDLER_H_
#define RTMP_RTMP_HANDLER_H_

#include <cstdint>
#include <string>
#include <vector>

#include "librtmp/rtmp.h"

#include "src/push/push_handler.h"
#include "src/rtmp/FlvAssist.h"

namespace tywebrtc {

// #include "user_basic_info_struct.h"

const uint32_t DEFAULT_TIMEOUT = 30;  // 30 seconds

class PeerConnection;

class RtmpHandler {
 public:
  explicit RtmpHandler(PeerConnection &pc);

  // note assignment and copy constructor
  ~RtmpHandler();

 public:
  int InitProtocolHandler(const std::string &rtmpUrl);
  bool InitSucc() const;

  int SendAudioFrame(const std::vector<char> &audioFrame, uint64_t frameMs);
  int SendVideoFrame(const std::vector<char> &h264Frame, uint64_t frameMs);

  // now no use
  int RecvRtmpPacket(std::vector<char> *o_recvBuf);

 private:
  int InitRtmp_();
  int SetupRtmp_(std::string rtmpUrl, bool writeable, uint32_t timeout,
                 bool liveSource);
  int ConnectRtmp_();

  int ReconnectRtmp_();

 private:
  // PeerConnection &belongingPeerConnection_;
  FlvAssist flvAssist;
  // uint32_t mMaxRtmpPacketLength = 0;

  std::string mRtmpUrl;
  bool mWriteable = false;
  uint32_t mTimeOut = 30;  // second
  bool mLiveSource = false;
  bool initSucc_ = false;

 public:
  // tmp public, indicate if rtmp setup is ok (not null)
  RTMP *mRtmpInstance = nullptr;
};

}  // namespace tywebrtc

#endif  // RTMP_RTMP_HANDLER_H_
