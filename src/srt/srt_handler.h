#ifndef SRT_SRT_HANDLER_H_
#define SRT_SRT_HANDLER_H_

#include <string>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif

#include "libavformat/avformat.h"

#ifdef __cplusplus
}
#endif

class SrtHandler {
 public:
  ~SrtHandler();

  int InitProtocolHandler(const std::string& srtUrl);
  bool InitSucc() const;

  int SendAudioFrame(const std::vector<char>& audioFrame, uint64_t frameMs);
  int SendVideoFrame(const std::vector<char>& videoFrame, uint64_t frameMs);

 private:
  int AddAudioStream_(uint32_t sampleRate, uint32_t channels,
                      AVDictionary*& options);
  int AddVideoStream_(uint32_t width, uint32_t height, AVDictionary*& options);

  int SendFrame_(const std::vector<char>& h264Frame, uint64_t frameMs,
                 bool bAudio);

 private:
  int audioStreamIndex_ = 0;
  int videooStreamIndex_ = 0;

 public:  // tmp
  AVFormatContext* formatContext_ = nullptr;
};

#endif  //  SRT_SRT_HANDLER_H_
