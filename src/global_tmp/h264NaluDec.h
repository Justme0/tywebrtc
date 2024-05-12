// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_GLOBAL_TMP_H264NALUDEC_H_
#define SRC_GLOBAL_TMP_H264NALUDEC_H_

namespace tywebrtc {

typedef enum WebVideoFrameType {
  WEB_VIDEO_FRAME_TYPE_I = 0x0,
  WEB_VIDEO_FRAME_TYPE_P = 0x1,
  WEB_VIDEO_FRAME_TYPE_P_WITHSP = 0x2,
  WEB_VIDEO_FRAME_TYPE_SP = 0x3,
  WEB_VIDEO_FRAME_TYPE_GF = 0x4,
  WEB_VIDEO_FRAME_TYPE_P_WITHB = 0x5,
  WEB_VIDEO_FRAME_TYPE_B = 0x6,
  WEB_VIDEO_FRAME_TYPE_P_MULTIREF = 0x7,  //急速模式下动态参考帧类型
  WEB_VIDEO_FRAME_TYPE_I_NoIDR = 0x8,  //解码段遇到这个后不清理参考帧列表
  WEB_VIDEO_FRAME_TYPE_BUTT
} WEB_VIDEO_FRAME_TYPE;

typedef struct Tag_bs_t {
  unsigned char *p_start;  //缓冲区首地址(这个开始是最低地址)
  unsigned char *p;        //缓冲区当前的读写指针
                           //当前字节的地址，这个会不断的++，每次++，进入一个新的字节
  unsigned char *p_end;  //缓冲区尾地址       //typedef unsigned char   uint8_t;
  int i_left;  // p所指字节当前还有多少 “位” 可读写 count number of
               // available(可用的)位
} bs_t;

WebVideoFrameType GetFrameType(unsigned char *pNalu, int Len);
}

#endif  // SRC_GLOBAL_TMP_H264NALUDEC_H_