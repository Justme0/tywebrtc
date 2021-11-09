#pragma once

#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <string>

// A simple and bad log util :)
// TODO: add log level
// why don't use glog: C++ style output is not friendly for reading.
// The best style is interpolation string style in many dynamic languages.
// In C++ the suboptimal style is C format string style.
inline void tylogWithMoreInfo(const char *fileName, int lineNumber,
                              const char *functionName, const char *format,
                              ...) {
  std::ofstream outfile;
  outfile.open("./log.txt",
               std::ios_base::app);  // append instead of overwrite,
                                     // maybe fail? TODO: roll log
                                     // and reserve recent N files

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
  outfile << "," << fileName;

  // * line number
  outfile << ":" << lineNumber << "L";

  // * function name
  outfile << "," << functionName << "()";

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

void tylog(const char *format, ...) __attribute__((format(printf, 1, 2)));
#define tylog(format, arg...) \
  tylogWithMoreInfo(__FILE__, __LINE__, __func__, format, ##arg);
