
#pragma once

#include "butt_enum.h"
#include "my_timer.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct in_t in_t;
typedef struct out_t out_t;
typedef struct gapi_hdl gapi_hdl;
typedef struct strobes_hdl strobes_hdl;

// внутреннее состояние модели
typedef struct state_t {
  // модули
  gapi_hdl *gapi;
  strobes_hdl *strobes;

  // входные данные
  const in_t *in;
  // выходные данные
  out_t *out;

  // текущий и предшествующий
  // отображаемый кадр
  uint8_t frame_current;
  uint8_t frame_prev;

  // channel (0-left; 1-right)
  uint8_t channel;

  // счетчик отрисованных кадров
  uint32_t frames_ctr;

  // runtime, обновляется раз в цикл
  // отрисовки (с)
  double runtime;

  // данные секундомера
  double prev_runtime;
  double my_timer_time;
  my_timer_mode_e my_timer_mode;

  bool meander; // 2,5 Гц

  struct {
    bool pressed[BUTTONS_COUNT];
    bool front[BUTTONS_COUNT];
    bool bfront[BUTTONS_COUNT];
    uint32_t time_press
        [BUTTONS_COUNT]; // (мс)
    uint32_t time_press_mem
        [BUTTONS_COUNT]; // (мс)

    int8_t wheels
        [WHEELS_COUNT]; // приращения
                        // крутилок
                        // (-1/0/+1)
  } butt_state;

  struct {
    uint16_t fps;
    uint32_t frame_counter;
  } render_data;

  // состояния модулей редактирования
  void *vv_num;
  void *vv_num_enc;
} state_t;

#ifdef __cplusplus
}
#endif
