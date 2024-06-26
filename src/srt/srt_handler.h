// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_SRT_SRT_HANDLER_H_
#define SRC_SRT_SRT_HANDLER_H_

#include <string>
#include <vector>

extern "C" {
#include "libavformat/avformat.h"
}

#include "src/push/push_handler.h"

namespace tywebrtc {

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

}  // namespace tywebrtc

#endif  //  SRC_SRT_SRT_HANDLER_H_
