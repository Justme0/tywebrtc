#include <unistd.h>

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "tylib/log/log.h"
#include "tylib/string/format_string.h"

extern "C" {
#include "libavformat/avformat.h"
}

#define av_err2string(errnum)                                          \
  av_make_error_string(                                                \
      static_cast<char *>(__builtin_alloca(AV_ERROR_MAX_STRING_SIZE)), \
      AV_ERROR_MAX_STRING_SIZE, errnum)

// OPT: not use global var
AVFormatContext *uplinkFileCtx_ = nullptr;
int audioStreamIndex_ = 0;
int videoStreamIndex_ = 0;

std::string input;
std::string output;

// tmp
inline int mkdir_p(const char *path, mode_t mode) {
  const char *p;
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

#define tylog(format, arg...)                  \
  MLOG_NORMAL(MLOG_DEF_LOGGER, format, ##arg); \
  printf(format, ##arg);                       \
  std::cout << std::endl

#define INIT_LOG_V2(path, format, level, size)                        \
  do {                                                                \
    mkdir_p(path, 0777);                                              \
    tylib::MLOG_INIT(MLOG_DEF_LOGGER, level, format, path, "", size); \
  } while (0)

int initFile() {
  INIT_LOG_V2("./log/",
              tylib::MLOG_F_TIME | tylib::MLOG_F_FILELINE | tylib::MLOG_F_FUNC,
              6, 150 * 1024 * 1024);

  avformat_alloc_output_context2(&uplinkFileCtx_, nullptr, "webm",
                                 output.data());

  if (nullptr == uplinkFileCtx_) {
    tylog("init uplinkFile fail, should not use assert :)");
    assert(!"init uplinkFile fail, should not use assert :)");
  }

  // Add video stream
  /*
  AVStream *video_stream = avformat_new_stream(uplinkFileCtx_, nullptr);
  if (!video_stream) {
    tylog("init uplinkFile fail, should not use assert :)");
    assert(!"init uplinkFile fail, should not use assert :)");
  }
  video_stream->codecpar->codec_id = AV_CODEC_ID_VP8;
  video_stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
  // OPT: use const or config
  video_stream->codecpar->width = 450;
  video_stream->codecpar->height = 450;
  videoStreamIndex_ = video_stream->index;
  tylog("video stream index=%d.", videoStreamIndex_);
  */

  // Add audio stream
  AVStream *audio_stream = avformat_new_stream(uplinkFileCtx_, nullptr);
  if (!audio_stream) {
    tylog("init uplinkFile fail, should not use assert :)");
    assert(!"init uplinkFile fail, should not use assert :)");
  }
  audio_stream->codecpar->codec_id = AV_CODEC_ID_OPUS;
  audio_stream->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
  audio_stream->codecpar->sample_rate = 48000;

  // OPT: parse OPUS stream to get channel number
  audio_stream->codecpar->ch_layout = AV_CHANNEL_LAYOUT_MONO;

  audioStreamIndex_ = audio_stream->index;
  tylog("audio stream index=%d.", audioStreamIndex_);

  // Open the output file
  assert(!(uplinkFileCtx_->oformat->flags & AVFMT_NOFILE));
  int ret =
      avio_open(&uplinkFileCtx_->pb, uplinkFileCtx_->url, AVIO_FLAG_WRITE);
  if (ret < 0) {
    tylog("cannot open avio, url=%s, ret=%d[%s]", uplinkFileCtx_->url, ret,
          av_err2string(ret));

    // use goto error?
    avformat_free_context(uplinkFileCtx_);
    uplinkFileCtx_ = nullptr;

    assert(!"init uplinkFile fail, should not use assert :)");
  }

  // Write the file header
  ret = avformat_write_header(uplinkFileCtx_, nullptr);
  if (ret < 0) {
    tylog("write header ret=%d[%s]", ret, av_err2string(ret));
    assert(!"init uplinkFile fail, should not use assert :)");
  }

  return 0;
}

const std::string kMediaTypeVideo = "video";
const std::string kMediaTypeAudio = "audio";

int WriteWebmFile(const std::string &frame, uint32_t pts,
                  const std::string &mediaType, bool bKeyFrame) {
  int ret = 0;

  int streamIndex = 0;
  AVRational timebase;
  if (kMediaTypeAudio == mediaType) {
    streamIndex = audioStreamIndex_;
  } else {
    streamIndex = videoStreamIndex_;
  }

  AVPacket avPacket;  // not alloc,may memory leak?
  av_init_packet(&avPacket);
  avPacket.data = reinterpret_cast<uint8_t *>(const_cast<char *>(frame.data()));
  avPacket.size = frame.size();
  if (bKeyFrame) {
    avPacket.flags |= AV_PKT_FLAG_KEY;
  }
  avPacket.stream_index = streamIndex;
  // avPacket.time_base = AVRational{1, 1000};

  avPacket.pts = pts;
  avPacket.dts = pts;

  tylog("pts=%u, size=%zu.", pts, frame.size());

  ret = av_interleaved_write_frame(uplinkFileCtx_, &avPacket);
  if (ret < 0) {
    tylog("vp8 interleaved_write_frame ret=%d[%s].", ret, av_err2string(ret));

    return ret;
  }

  return 0;
}

int main(int argc, char **argv) {
  int opt;

  // Loop through the arguments and process them using getopt.
  while ((opt = getopt(argc, argv, "i:o:")) != -1) {
    switch (opt) {
      case 'i':
        input = optarg;
        break;
      case 'o':
        output = optarg;
        break;
      case '?':  // Option not recognized or missing argument
        if (optopt == 'i' || optopt == 'o') {
          std::cerr << "Option -" << char(optopt) << " requires an argument."
                    << std::endl;
        } else {
          std::cerr << "Unknown option: -" << char(optopt) << std::endl;
        }
        return 1;
      default:
        std::cerr << "Usage: " << argv[0] << " -i <input> -o <output>"
                  << std::endl;
        return 1;
    }
  }

  std::ifstream ifs(input);

  if (!ifs) {
    tylog("ifs null");
    return -1;
  }

  std::vector<char> data = std::vector<char>(
      std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
  int ret = initFile();
  assert(ret == 0);

  tylog("init succ");

  uint32_t itemLen = 0;
  int pts = 0;
  for (char *p = data.data(); p != data.data() + data.size();
       p += 4 + itemLen) {
    itemLen = *reinterpret_cast<uint32_t *>(p);
    WriteWebmFile(std::string(p + 4, p + 4 + itemLen), pts, kMediaTypeAudio,
                  false);
    pts += 20;
  }

  av_write_trailer(uplinkFileCtx_);
  avio_close(uplinkFileCtx_->pb);
  avformat_free_context(uplinkFileCtx_);
}