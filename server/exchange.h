#pragma once

#include <stdint.h>

#include "controls.h"

#define RTM_HEADER 0xAABB
#define CTM_HEADER 0xCCDD

#ifdef __cplusplus
extern "C" {
#endif

typedef struct render_to_model_t {
  uint16_t header; // 0xAABB
  uint16_t fps;
  uint16_t frame_counter;
  uint16_t reserve[64];
} render_to_model_t;

typedef enum phy_line_id_e {
  PHY_LINE_ETH = 0,

  PHY_LINE_COUNT,
} phy_line_id_e;

typedef struct line_stat_t {
  int len_err;
  int cnt_err;
  int crc_err;
} line_stat_t;

typedef struct controls_to_model_t {
  uint16_t header; // 0xCCDD;
  controls_t controls;
} controls_to_model_t;

#ifdef __cplusplus
}
#endif
