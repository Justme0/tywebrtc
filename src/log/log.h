#pragma once

#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <string>

// A simple and bad log util :)
// TODO: add log level
// why not use glog: C++ style output is not friendly for reading.
// The best style is interpolation string style in many dynamic languages.
// In C++ the suboptimal style is C format string style.

void tylog(const char* format, ...) __attribute__((format(printf,1,2)));
inline void tylog(const char* format, ...) {
    std::ofstream outfile;
    system("echo shitt >> a.txt");

    outfile.open("./log.txt", std::ios_base::app); // append instead of overwrite

    char buf[8 * 1024];

    va_list args;
    va_start(args, format);
    int n = vsnprintf(buf, sizeof(buf), format, args);
    if (n>=(int)sizeof(buf)) {
        n=sizeof(buf)-1;
    }
    va_end(args);

    buf[n++]='\n'; 

    outfile << time(NULL) << " " << std::string(buf, n);; // to use printf("%.*s", len, str); style
}
