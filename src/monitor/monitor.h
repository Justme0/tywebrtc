// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_MONITOR_MONITOR_H_
#define SRC_MONITOR_MONITOR_H_

#include <map>
#include <memory>
#include <string>

#include "prometheus/family.h"
#include "prometheus/gauge.h"
#include "prometheus/registry.h"

namespace tywebrtc {
extern std::shared_ptr<prometheus::Registry> g_pRegistry;

void DimensionAdd(const std::map<std::string, std::string>& dimensions,
                  const std::string& attributeName, int value);
}

#endif  // SRC_MONITOR_MONITOR_H_