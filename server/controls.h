#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "butt_enum.h"
#include "exchange.h"

#ifdef __cplusplus
extern "C" {
#endif

// состояние органов управления (кнопки,
// крутилки)
typedef struct controls_t {
  bool butt_pressed[BUTTONS_COUNT];
  uint16_t wheel_position[WHEELS_COUNT];
} controls_t;

#ifdef __cplusplus
}
#endif
