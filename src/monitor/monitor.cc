// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/monitor/monitor.h"

namespace tywebrtc {

std::shared_ptr<prometheus::Registry> g_pRegistry;

void DimensionAdd(const std::map<std::string, std::string>& dimensions,
                  const std::string& attributeName, int value) {
  prometheus::BuildGauge()
      .Name(attributeName.data())
      .Help(attributeName.data())
      .Register(*g_pRegistry)
      .Add(dimensions)
      .Increment(value);
}

// have bug
// attribute =>
std::map<std::string, prometheus::Family<prometheus::Gauge>*> g_mSet;

void DimensionSet(const std::map<std::string, std::string>& dimensions,
                  const std::string& attributeName, int value) {
  prometheus::Family<prometheus::Gauge>* family = g_mSet[attributeName];
  if (nullptr != family) {
    g_pRegistry->Remove(*family);
  }

  // save it for delete next time
  family = &prometheus::BuildGauge()
                .Name(attributeName.data())
                .Help(attributeName.data())
                .Register(*g_pRegistry);

  family->Add(dimensions).Set(value);
}
}  // namespace tywebrtc
