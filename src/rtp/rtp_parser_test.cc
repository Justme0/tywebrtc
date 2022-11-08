/*
 *  Copyright (c) 2022 The tywebrtc project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a MIT license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtp/rtp_parser.h"

#include "gtest/gtest.h"

TEST(RtpParserTest, PowerSeq) {
  auto p = SplitPowerSeq(0);
  EXPECT_EQ(p.first, 0);
  EXPECT_EQ(p.second, 0);

  p = SplitPowerSeq((-1) << 16 | 0xFFFF);
  EXPECT_EQ(p.first, -1);
  EXPECT_EQ(p.second, 0xFFFF);

  p = SplitPowerSeq((50) << 16 | 0xABCD);
  EXPECT_EQ(p.first, 50);
  EXPECT_EQ(p.second, 0xABCD);
}
