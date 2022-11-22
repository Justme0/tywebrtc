// A poor but simple log

#ifndef LOG_LOG_H_
#define LOG_LOG_H_

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "tylib/log/log.h"
#include "tylib/time/timer.h"

// https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po
// inline std::string ExecLinuxCmd(const char *cmd) {
//   std::array<char, 128> buffer;
//   std::string result;
//   std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
//   if (!pipe) {
//     throw std::runtime_error("popen() failed!");
//   }
//   while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
//     result += buffer.data();
//   }
//   return result;
// }

// to move to tylib
// C++17 can use `std::filesystem::file_size`
inline std::ifstream::pos_type filesize(const std::string &filename) {
  std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
  return in.tellg();
}

// to optimize performance
// inline std::string HandleFileSize() {
//   const int kReserveRecentLogFileNumber = 10;
//   const int64_t kMaxSingleFileByte = 200 * 1024 * 1024;
//
//   // should in config file
//   int64_t size = filesize(g_kLogFile);
//   // move to config file
//   if (size >= g_kMaxSingleFileByte) {
//     system("mv ./tywebrtc.log ./tywebrtc.log.1");  // to use config and cross
//                                                    // platform
//   }
//
//   struct tm t;
//   time_t nowSec = g_now_ms / 1000;
//   localtime_r(&nowSec, &t);
//   char tmp[1024];
//   snprintf(tmp, sizeof(tmp), "tywebrtc_%4d%02d%02d_%02d%02d%02d.log",
//            t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min,
//            t.tm_sec);
// }

// get last file name, not path
inline const char *CleanFileName(const char *fileName) {
  const char *pos = strrchr(fileName, '/');
  if (pos != nullptr) {
    return pos + 1;
  } else {
    return fileName;
  }
}

// A simple and bad log util :)
// TODO: add log level
// why don't use glog: C++ style output is not friendly for reading.
// The best style is interpolation string style in many dynamic languages.
// In C++ the suboptimal style is C format string style.
void tylogWithMoreInfo(const char *fileName, int lineNumber,
                       const char *functionName, const char *format, ...)
    __attribute__((format(printf, 4, 5)));

inline void tylogWithMoreInfo(const char *fileName, int lineNumber,
                              const char *functionName, const char *format,
                              ...) {
  // HandleFileSize();

  std::ofstream outfile;
  // append instead of overwrite, maybe fail?
  // why append mode not work?

  const std::string &g_kLogFile = "./tywebrtc.log";
  outfile.open(g_kLogFile, std::ios_base::app);

  // taylor : move get now time util to tylib
  // * time
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);

  // get sec(tm struct presentation), ms, us
  struct tm tm;
  localtime_r(&t.tv_sec, &tm);

  int ms = t.tv_nsec / 1000000;
  int us = t.tv_nsec / 1000 - ms * 1000;
  // int ns = t.tv_nsec - ms * 1000000 - us * 1000;

  char timeBuffer[50];
  snprintf(timeBuffer, sizeof(timeBuffer),
           "%4d-%02d-%02d %02d:%02d:%02d.%03d%03d", tm.tm_year + 1900,
           tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms, us);

  outfile << timeBuffer;

  // * filename
  outfile << " " << CleanFileName(fileName);

  // * line number
  outfile << ":" << lineNumber;

  // * function name
  outfile << " " << functionName;

  // * user's content
  const int kUserContentMaxLengthByte = 8 * 1024;
  char buf[kUserContentMaxLengthByte];

  va_list args;
  va_start(args, format);
  int n = vsnprintf(buf, sizeof(buf), format, args);
  if (n >= (int)sizeof(buf)) {
    n = sizeof(buf) - 1;
  }
  va_end(args);

  buf[n++] = '\n';

  outfile << " " << std::string(buf, n);  // to use printf("%.*s", len, str)
                                          // style to escape string copy
}

// temp reserve last 10 log files
#define tylog(format, arg...) MLOG_NORMAL(MLOG_DEF_LOGGER, format, ##arg)

// very poor preformance:
// system("ls -tr log/tywebrtc*log | head -n -20 | xargs rm -f")

// mlogWithMoreInfo(__FILE__, __LINE__, __func__, format, ##arg)

#define tylogAndPrintfln(format, arg...)       \
  MLOG_NORMAL(MLOG_DEF_LOGGER, format, ##arg); \
  printf(format, ##arg);                       \
  std::cout << std::endl

#endif  // LOG_LOG_H_
