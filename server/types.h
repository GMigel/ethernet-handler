#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct minmax_t {
  float min;
  float max;
} minmax_t;

typedef struct param_t {
  float value;
  bool valid;
  bool display;
} param_t;

typedef struct paramd_t {
  double value;
  bool valid;
  bool display;
} paramd_t;

typedef struct params_t {
  char value[17];
  bool valid;
  bool display;
} params_t;

typedef struct ppm_param_t {
  uint8_t num;
  char name[8];
  bool valid;
  bool display;
} ppm_param_t;

typedef enum health_state_e {
  HEALTH_STATE_UNKNOWN = 0,

  HEALTH_STATE_OK,
  HEALTH_STATE_WARN,
  HEALTH_STATE_ERR,

  HEALTH_STATE_ON_1,
  HEALTH_STATE_ON_2,
  HEALTH_STATE_ON_3,
  HEALTH_STATE_OFF_1,
  HEALTH_STATE_OFF_2,
  HEALTH_STATE_OFF_3,

  HEALTH_STATE_COUNT
} health_state_e;

typedef struct param_health_t {
  health_state_e health;
  bool valid;
  bool display;
} param_health_t;

void param_t_assign(param_t* param, float value, bool valid, bool display);
void param_t_assign_def(param_t* param, float value);
bool param_t_draw(const param_t* param);

void paramh_t_assign(param_health_t* param, int value, bool valid,
                     bool display);
void paramh_t_assign_def(param_health_t* param, bool val);
bool paramh_t_draw(const param_health_t* param);

void paramd_t_assign(paramd_t* param, double value, bool valid, bool display);
void paramd_t_assign_def(paramd_t* param, double value);
bool paramd_t_draw(const paramd_t* param);

void params_t_assign(params_t* param, char* value, bool valid, bool display);
bool params_t_draw(const params_t* param);

typedef struct signal_t {
  bool signal;
  bool valid;
} signal_t;

void signal_t_assign_def(signal_t* signal, bool val);

typedef enum pressure_type_t {
  QFE = 0,
  QNH = 1,
  QNE = 2,
  STD = QNE,
  PRESSURE_COUNT,
} pressure_type_t;

typedef enum heading_display_t {
  DISPLAY_MAGNETIC_HEADING = 0,
  DISPLAY_TRUE_HEADING = 1
} heading_display_t;

void heading_display_t_assign(heading_display_t* param, uint8_t value);

typedef struct date_t {
  uint8_t day;
  uint8_t month;
  uint16_t year;
} date_t;

typedef struct ttime_t {
  uint8_t ss;
  uint8_t mm;
  uint8_t hh;
} ttime_t;

#pragma pack(push, 1)

typedef struct me104_fram_state_t {
  bool disable_fpga_status;  // Отключить индикацию статуса МЭ поверх кадра
  bool enable_color_profile;  // Отключить цветовой профиль дисплея
  bool enable_auto_heat;  // Разрешить автоматическое включение подогрева
  bool enable_block_heat;  // Разрешить блокировку подогрева

  bool display_18;  // Тип дисплея 18 бит
  bool display_24;  // Тип дисплея 24 бит
  bool vesa_in;     // Тип входного сигнала Vesa
  bool jeida_in1;   // Тип входного сигнала Jeida

  int16_t me104_manufacture_id;  // Зав.№ МЭ-104.01

  uint8_t status_y;  // Координата Y индикатора статуса ПЛИС на дисплее
                     // (Смещение от правого нижнего угла от 0 до 255)
  uint8_t status_x;  // Координата X индикатора статуса ПЛИС на дисплее
                     // (Смещение от правого нижнего угла от 0 до 255)

  int16_t brightness_keyboard;  // Текущее заданное значение яркости кнопок
  int16_t brightness_display;  // Текущее заданное значение яркости дисплея
  uint16_t min_keyboard_brightness_day;  // Минимальная яркость подсветки
                                         // клавиатуры (День)
  uint16_t max_keyboard_brightness_day;  // Максимальная яркость подсветки
                                         // клавиатуры (День)
  uint16_t min_display_brightness_day;  // Минимальная яркость подсветки матрицы
                                        // (День)
  uint16_t max_display_brightness_day;  // Максимальная яркость подсветки
                                        // матрицы (День)
  uint16_t min_keyboard_brightness_night;  // Минимальная яркость подсветки
                                           // клавиатуры (Ночь)
  uint16_t max_keyboard_brightness_night;  // Максимальная яркость подсветки
                                           // клавиатуры (Ночь)
  uint16_t min_display_brightness_night;  // Минимальная яркость подсветки
                                          // матрицы (Ночь)
  uint16_t max_display_brightness_night;  // Максимальная яркость подсветки
                                          // матрицы (Ночь)

  uint16_t r_x;  // Матрица преобразования цвета. Коэффициент  r_x
  uint16_t r_y;  // Матрица преобразования цвета. Коэффициент  r_y
  uint16_t r_z;  // Матрица преобразования цвета. Коэффициент  r_z
  uint16_t g_x;  // Матрица преобразования цвета. Коэффициент  g_x
  uint16_t g_y;  // Матрица преобразования цвета. Коэффициент  g_y
  uint16_t g_z;  // Матрица преобразования цвета. Коэффициент  g_z
  uint16_t b_x;  // Матрица преобразования цвета. Коэффициент  b_x
  uint16_t b_y;  // Матрица преобразования цвета. Коэффициент  b_y
  uint16_t b_z;  // Матрица преобразования цвета. Коэффициент  b_z

  float
      heat_temperature_on;  // Температура включения автоподогрева      // 1/128
  float heat_temperature_off;  // Температура выключения автоподогрева     //
                               // 1/128
  float heat_temperature_max;  // Предельная температура работы подогрева  //
                               // 1/128
} me104_fram_state_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif
