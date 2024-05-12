// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_RTMP_DOMAINRESOLVE_H_
#define SRC_RTMP_DOMAINRESOLVE_H_

// #include <inttypes.h>
#include <netinet/in.h>
#include <sys/types.h>

#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace tywebrtc {

class DomainResolve {
 public:
  DomainResolve()
      : m_TimeoutMs(200),
        m_IntervalMs(1000){

        };

  static DomainResolve *getInstance() {
    static DomainResolve instance;
    return &instance;
  }

  int init(uint32_t TimeoutMs, uint32_t IntervalMs);

  //域名解析失败，不启动，cmlb兜底只在定时刷新时使用
  bool addDomain(const std::string &domain);

  bool getAddrByDomain(const std::string &domain, struct sockaddr_in &addr);

  void refreshDomain(uint32_t now);

 private:
  std::map<std::string, sockaddr_in> domainMap;
  uint32_t m_TimeoutMs;
  uint32_t m_IntervalMs;
};
}

#endif  // SRC_RTMP_DOMAINRESOLVE_H_
