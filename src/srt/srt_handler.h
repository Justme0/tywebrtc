#ifndef SRT_SRT_HANDLER_H_
#define SRT_SRT_HANDLER_H_

#include <string>
#include <vector>

extern "C" {
#include "libavformat/avformat.h"
}

#include "push/push_handler.h"

// create SRT server:
// ffmpeg -loglevel debug -f mpegts -i srt://127.0.0.1:9001?mode=listener -c
// copy a.aac -y
class SrtHandler : public PushHandler {
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

  int SendFrame_(const std::vector<char>& frame, uint64_t frameMs, bool bAudio);

 private:
  int audioStreamIndex_ = 0;
  int videooStreamIndex_ = 0;

 public:  // tmp
  AVFormatContext* formatContext_ = nullptr;
};

#endif  //  SRT_SRT_HANDLER_H_
