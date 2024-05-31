// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/global_tmp/global_tmp.h"

#include "gtest/gtest.h"

namespace tywebrtc {

TEST(NtpTimeTest, NtpMsConvert) {
  for (auto ms : {0, 1, 2, 1900, static_cast<int>(time(nullptr))}) {
    EXPECT_EQ(MsToNtp(ms).ToUnixMs_(), ms);
  }

  // must 1ULL otherwise overflow
  EXPECT_EQ(NtpTime(2208988800 * (1ULL << 32)).ToUnixMs_(), 0);
}

}  // namespace tywebrtc
