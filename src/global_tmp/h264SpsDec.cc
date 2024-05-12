// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include <cstdio>
#include <cstdlib>

namespace tywebrtc {

/**
    bit buffer handling
*/
struct bit_buffer {
  const char *start;
  /*size_t size;*/
  const char *current;
  unsigned char read_bits;
};

static void skip_bits(bit_buffer *bb, size_t nbits) {
  bb->current = bb->current + ((nbits + bb->read_bits) / 8);
  bb->read_bits = static_cast<unsigned char>((bb->read_bits + nbits) % 8);
}

static unsigned char get_bit(bit_buffer *bb) {
  unsigned char ret = 0;
  ret = (*(bb->current) >> (7 - bb->read_bits)) & 0x1;
  if (bb->read_bits == 7) {
    bb->read_bits = 0;
    bb->current++;
  } else {
    bb->read_bits++;
  }
  return ret;
}

static unsigned int get_bits(bit_buffer *bb, size_t nbits) {
  unsigned int i, ret;
  ret = 0;
  for (i = 0; i < nbits; i++) {
    ret = (ret << 1) + get_bit(bb);
  }
  return ret;
}

static unsigned int exp_golomb_ue(bit_buffer *bb) {
  unsigned char bit, significant_bits;
  significant_bits = 0;
  bit = get_bit(bb);
  while (bit == 0) {
    if (32 < significant_bits) {
      //防止非法输入导致的内存读越界
      printf("big error in sps parse, significant_bits:%u >20\n",
             significant_bits);
      return 0;
    }

    // printf("significant_bits:%u\n", significant_bits);
    significant_bits++;
    bit = get_bit(bb);
  }
  return (1 << significant_bits) + get_bits(bb, significant_bits) - 1;
}

static int exp_golomb_se(bit_buffer *bb) {
  int ret;
  ret = exp_golomb_ue(bb);
  if ((ret & 0x1) == 0) {
    return -(ret >> 1);
  } else {
    return (ret + 1) >> 1;
  }
}

static void parse_scaling_list(unsigned int size, bit_buffer *bb) {
  unsigned int last_scale, next_scale, i;
  int delta_scale;
  last_scale = 8;
  next_scale = 8;
  for (i = 0; i < size; i++) {
    if (next_scale != 0) {
      delta_scale = exp_golomb_se(bb);
      next_scale = (last_scale + delta_scale + 256) % 256;
    }
    if (next_scale != 0) {
      last_scale = next_scale;
    }
  }
}

/**
    Parses a SPS NALU to retrieve video width and height
*/
void H264ParseSps(const char *sps, unsigned int, int &width, int &height) {
  bit_buffer bb;
  unsigned int profile, pic_order_cnt_type, width_in_mbs, height_in_map_units;
  unsigned int i, size, left, right, top, bottom;
  unsigned char frame_mbs_only_flag;

  bb.start = sps;
  /*bb.size = sps_size;*/
  bb.current = sps;
  bb.read_bits = 0;

  /* skip first byte, since we already know we're parsing a SPS */
  skip_bits(&bb, 8);
  /* get profile */
  profile = get_bits(&bb, 8);
  /* skip 4 bits + 4 zeroed bits + 8 bits = 16 bits = 2 bytes */
  skip_bits(&bb, 16);

  /* read sps id, first exp-golomb encoded value */
  exp_golomb_ue(&bb);

  if (profile == 100 || profile == 110 || profile == 122 || profile == 144) {
    /* chroma format idx */
    if (exp_golomb_ue(&bb) == 3) {
      skip_bits(&bb, 1);
    }
    /* bit depth luma minus8 */
    exp_golomb_ue(&bb);
    /* bit depth chroma minus8 */
    exp_golomb_ue(&bb);
    /* Qpprime Y Zero Transform Bypass flag */
    skip_bits(&bb, 1);
    /* Seq Scaling Matrix Present Flag */
    if (get_bit(&bb)) {
      for (i = 0; i < 8; i++) {
        /* Seq Scaling List Present Flag */
        if (get_bit(&bb)) {
          parse_scaling_list(i < 6 ? 16 : 64, &bb);
        }
      }
    }
  }
  /* log2_max_frame_num_minus4 */
  exp_golomb_ue(&bb);
  /* pic_order_cnt_type */
  pic_order_cnt_type = exp_golomb_ue(&bb);
  if (pic_order_cnt_type == 0) {
    /* log2_max_pic_order_cnt_lsb_minus4 */
    exp_golomb_ue(&bb);
  } else if (pic_order_cnt_type == 1) {
    /* delta_pic_order_always_zero_flag */
    skip_bits(&bb, 1);
    /* offset_for_non_ref_pic */
    exp_golomb_se(&bb);
    /* offset_for_top_to_bottom_field */
    exp_golomb_se(&bb);
    size = exp_golomb_ue(&bb);
    for (i = 0; i < size; i++) {
      /* offset_for_ref_frame */
      exp_golomb_se(&bb);
    }
  }
  /* num_ref_frames */
  exp_golomb_ue(&bb);
  /* gaps_in_frame_num_value_allowed_flag */
  skip_bits(&bb, 1);
  /* pic_width_in_mbs */
  width_in_mbs = exp_golomb_ue(&bb) + 1;
  /* pic_height_in_map_units */
  height_in_map_units = exp_golomb_ue(&bb) + 1;
  /* frame_mbs_only_flag */
  frame_mbs_only_flag = get_bit(&bb);
  if (!frame_mbs_only_flag) {
    /* mb_adaptive_frame_field */
    skip_bits(&bb, 1);
  }
  /* direct_8x8_inference_flag */
  skip_bits(&bb, 1);
  /* frame_cropping */
  left = right = top = bottom = 0;
  if (get_bit(&bb)) {
    left = exp_golomb_ue(&bb) * 2;
    right = exp_golomb_ue(&bb) * 2;
    top = exp_golomb_ue(&bb) * 2;
    bottom = exp_golomb_ue(&bb) * 2;
    if (!frame_mbs_only_flag) {
      top *= 2;
      bottom *= 2;
    }
  }
  /* width */
  width = width_in_mbs * 16 - (left + right);
  /* height */
  height = height_in_map_units * 16 - (top + bottom);
  if (!frame_mbs_only_flag) {
    height *= 2;
  }
}

}  // namespace tywebrtc
