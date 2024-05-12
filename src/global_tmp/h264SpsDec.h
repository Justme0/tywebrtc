// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_GLOBAL_TMP_H264SPSDEC_H_
#define SRC_GLOBAL_TMP_H264SPSDEC_H_

namespace tywebrtc {
void H264ParseSps(const char *sps, unsigned int sps_size, int &width,
                  int &height);
}

#endif  // SRC_GLOBAL_TMP_H264SPSDEC_H_