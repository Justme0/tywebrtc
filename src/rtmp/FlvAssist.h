#ifndef __FLV_ASSIST_H__
#define __FLV_ASSIST_H__

#include <arpa/inet.h>

#include <vector>

#include "src/rtmp/MediaBuffer.h"

namespace tywebrtc {

#define AAC_ADTS_HEADER_LENGTH 7
#define DST_BUF_LEN (2048 * 1024)

class RtmpHandler;

class FlvAssist {
 public:
  FlvAssist(RtmpHandler& belongingRtmpHandler);

  int SendVideoFrame(const std::vector<char>& h264Frame, uint64_t frameMs);
  int SendAudioFrame(const std::vector<char>& audioFrame, uint64_t frameMs);

  FlvVideoFrameType AvcFrameType2FlvVideoFrameType(
      const enVideoH264NaluType avcFrameType);
  FlvAacProfile AacProfile2FlvAacProfile(const Mpeg2AacProfile aacProfile);

  int toAvc(MediaBuffer& mediaBuffer);
  int CheckNalu(unsigned char* buffer, unsigned int buffer_len);
  bool isSei(unsigned char type);
  bool isSps(unsigned char type);
  bool isPps(unsigned char type);

  int makeAvcConfTag(MediaBuffer& mediaBuffer, int& tagSize);
  int makeAacConfTag(MediaBuffer& mediaBuffer);

  void Init();

 private:
  int makeTag(MediaBuffer& mediaBuffer, MediaType mediaType);
  int makeAvcTag(MediaBuffer& mediaBuffer);
  int makeAacTag(MediaBuffer& mediaBuffer);

 public:
  RtmpHandler& belongingRtmpHandler_;

  static const unsigned char sFrameStartCode[];
  static const unsigned char sSliceStartCode[];
  unsigned int mStartTimeAud;
  unsigned int mStartTimeVid;
  bool mVideoConfigTag;  // 是否已有config tag
  bool mAudioConfigTag;  // 是否已有config tag

  unsigned char m_LastSampleRate;
  unsigned int mAudioFramecnt;
  H264Context mContext;
};

}  // namespace tywebrtc

#endif  // __MEDIA_FLV_H__
