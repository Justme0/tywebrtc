#ifndef LOG_LOG_H_
#define LOG_LOG_H_

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

// to move to tylib
// C++17 can use `std::filesystem::file_size`
inline std::ifstream::pos_type filesize(const std::string &filename) {
  std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
  return in.tellg();
}

// to optimize performance
inline void HandleFileSize() {
  // should in config file
  const std::string &g_kLogFile = "./log.txt";
  int64_t size = filesize(g_kLogFile);
  const int64_t g_kMaxSingleFileByte = 100 * 1024 * 1024;
  if (size >= g_kMaxSingleFileByte) {
    system("mv ./log.txt ./log.txt.1");  // to use config and cross platform
  }
}

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
  HandleFileSize();

  std::ofstream outfile;
  // append instead of overwrite, maybe fail?
  // TODO: roll log and reserve recent N files
  // why append mode not work?
  const std::string &g_kLogFile = "./log.txt";
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
           "%4d-%02d-%02d %02d:%02d:%02d.%03d %03d", tm.tm_year + 1900,
           tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms, us);

  outfile << timeBuffer;

  // * filename
  outfile << " " << CleanFileName(fileName);

  // * line number
  outfile << ":" << lineNumber;

  // * function name
  outfile << " " << functionName << "()";

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

#define tylog(format, arg...) \
  tylogWithMoreInfo(__FILE__, __LINE__, __func__, format, ##arg);

#define tylogAndPrintfln(format, arg...)                          \
  tylogWithMoreInfo(__FILE__, __LINE__, __func__, format, ##arg); \
  printf(format, ##arg);                                          \
  std::cout << std::endl;

#endif  // LOG_LOG_H_
