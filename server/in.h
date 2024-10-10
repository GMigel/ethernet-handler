#pragma once

#include "exchange.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct brightness_t {
  int16_t display;
  int16_t keyboard;
  bool day_night;
  bool dirty;
} brightness_t;

typedef struct me104_state_t {
  bool eth_link_valid;
  bool spw0_link_valid;
  bool spw1_link_valid;

  line_stat_t line_stat[PHY_LINE_COUNT];

  //-------------------------------СДС0----------------------------------------
  bool me104_ps422_p1_valid; // bit 1
  bool me104_ps422_p2_valid; // bit 2
  //---------------------------------------------------------------------------

  //-------------------------------СДС1----------------------------------------
  bool me_104_01_valid;       // bit 1
  bool fpga_config_valid;     // bit 2
  bool fpga_system_bus_valid; // bit 6
  bool fpga_bit_valid;        // bit 7
  bool fram_valid;            // bit 8
  bool i2c_valid;             // bit 9
  bool keyboard_valid;        // bit 10
  bool dvo1_link_valid;       // bit 11
  bool dvo2_link_valid;       // bit 12
  bool dt1_link_valid;        // bit 13
  bool dt2_link_valid;        // bit 14
  bool dt3_link_valid;        // bit 15
  bool dt4_link_valid;        // bit 16
  //---------------------------------------------------------------------------

  //-------------------------------СДС2----------------------------------------
  bool mv_valid;            // bit 1
  bool mv_link_valid;       // bit 2
  bool mg_valid;            // bit 3
  bool mg_link_valid;       // bit 4
  bool video_in_valid;      // bit 5
  bool video_in_freq_valid; // bit 6
  bool video_out_valid;     // bit 7
  bool data_from_mg;        // bit 9
  bool data_from_mv;        // bit 10
  bool main_comp_valid;     // bit 11
  bool color_profile_on;    // bit 13
  bool me104_status_on;     // bit 14
  bool color_profile_valid; // bit 15
  bool data_from_fram;      // bit 16
  //---------------------------------------------------------------------------

  bool lvds1_link_valid;
  bool lvds2_link_valid;

  int16_t me104_software_version;
  uint32_t me104_software_crc;

  int16_t me104_manufacture_id;

  int16_t start_optime_hh;
  int16_t start_optime_mm;
  int16_t start_optime_ss;

  uint16_t full_optime_hh;
  uint16_t full_optime_mm;
  uint16_t full_optime_ss;

  int16_t m01_pos;
  int16_t m02_pos;
  int16_t m11_pos;
  int16_t m12_pos;

  uint16_t buttons_sds4;
  uint16_t buttons_sds5;

  float luminance_d1; // Текущее значение освещенности с датчика №1
  float luminance_d2; // Текущее значение освещенности с датчика №2

  float temperature_d1; // Температура лицевой панели МФИ датчик №1
  float temperature_d2; // Температура лицевой панели МФИ датчик №2
  float temperature_d3; // Температура лицевой панели МФИ датчик №3
  float temperature_d4; // Температура лицевой панели МФИ датчик №4

  uint16_t min_keyboard_brightness_day; // Текущая минимальная яркость подсветки клавиатуры(День)
  uint16_t max_keyboard_brightness_day; // Текущая максимальная яркость подсветки клавиатуры(День)
  uint16_t min_keyboard_brightness_night; // Текущая минимальная яркость подсветки клавиатуры(Ночь)
  uint16_t max_keyboard_brightness_night; // Текущая максимальная яркость подсветки клавиатуры(Ночь)
  uint16_t min_display_brightness_day; // Текущая минимальная яркость подсветки дисплея(День)
  uint16_t max_display_brightness_day; // Текущая максимальная яркость подсветки дисплея(День)
  uint16_t min_display_brightness_night; // Текущая минимальная яркость подсветки дисплея(Ночь)
  uint16_t max_display_brightness_night; // Текущая максимальная яркость подсветки дисплея(Ночь)

  bool flash_write_done;

  bool heat_on;

  // 2
  union {
    uint16_t w;
    struct {
      unsigned mv_valid : 1;       // Исправность МВ
      unsigned mv_link_valid : 1;  // Исправность связи с МВ
      unsigned mg_valid : 1;       // Исправность МГ
      unsigned mg_link_valid : 1;  // Исправность связи с МГ
      unsigned video_in_valid : 1; // Исправность входного видеосигнала
      unsigned video_in_freq_valid : 1; // Исправность тактовой частоты вх. видеосигнала
      unsigned video_out_valid : 1; // Исправность выходного видеосигнала
      unsigned bit8 : 1;
      unsigned data_from_mg : 1;    // Прием данных идет от МГ
      unsigned data_from_mv : 1;    // Прием видео идет от МВ
      unsigned main_comp_valid : 1; // Исправность ведущего вычислителя
      unsigned bit12 : 1;
      unsigned lcd_off : 1;      // Подсветка матрицы выключена
      unsigned keyboard_off : 1; // Подсветка клавиатуры выключена
      unsigned sld_off : 1;      // Подсветка SLD выключена
      unsigned heat_off : 1;     // Подогрев матрицы выключен
    } bit;
  } sds2;
  // 3
  union {
    uint16_t w;
    struct {
      unsigned mv_no_data : 1;      // МВ. Нет данных
      unsigned mv_crc_err : 1;      // МВ. Ошибка CRC
      unsigned mv_crc_err_last : 1; // МВ. Ошибка CRC за последнюю секунду
      unsigned mv_data_update_err : 1; // МВ. Ошибка обновления данных для передатчика
      unsigned mg_no_data : 1;         // МГ. Нет данных
      unsigned mg_crc_err : 1;         // МГ. Ошибка CRC
      unsigned mg_crc_err_last : 1; // МГ. Ошибка CRC за последнюю секунду
      unsigned mg_data_update_err : 1; // МГ. Ошибка обновления данных для передатчика
      unsigned fram_read_done : 1;     // FRAM. Чтение выполнено
      unsigned fram_write_done : 1;     // FRAM. Запись выполнена
      unsigned fram_header_err : 1;     // FRAM. Ошибка заголовка пакета параметров
      unsigned fram_crc_err : 1;        // FRAM. Ошибка CRC пакета параметров
      unsigned fram_optime_err : 1;     // FRAM. Ошибка времени наработки
      unsigned fram_id_err : 1;         // FRAM. Ошибка FRAM ID
      unsigned fram_optime_err_fix : 1; // FRAM. Исправлена ошибка времени наработки
      unsigned fram_ready : 1;          // FRAM. Готовность к работе
    } bit;
  } sds3;

  uint16_t mv_rs422_pack1_counter;         // Счетчик  вх. пакетов RS422 № 1 от МВ
  uint16_t mg_rs422_pack1_counter;         // Счетчик  вх. пакетов RS422 № 1 от МГ
  uint16_t mv_rs422_pack2_counter;         // Счетчик  вх. пакетов RS422 № 2 от МВ
  uint16_t mg_rs422_pack2_counter;         // Счетчик  вх. пакетов RS422 № 2 от МГ
  uint16_t mv_rs422_crc_err_counter;       // Счетчик ошибок CRC вх. пакета RS422 от МВ
  uint16_t mg_rs422_crc_err_counter;       // Счетчик ошибок CRC вх. пакета RS422 от МГ
  uint16_t rs422_pack1_rx_err_counter;     // Счётчик ошибок приёма пакета 1 от МЭ
  uint16_t rs422_pack2_rx_err_counter;     // Счётчик ошибок приёма пакета 2 от МЭ
  uint16_t rs422_pack1_rx_err_crc_counter; // Счётчик ошибок CRC пакета 1 от МЭ
  uint16_t rs422_pack2_rx_err_crc_counter; // Счётчик ошибок CRC пакета 2 от МЭ
  uint16_t rs422_tx_err_counter; // Счётчик ошибок "Исправность связи с МГ/МВ"

  me104_fram_state_t me104_fpga_fram_data;

} me104_state_t;

typedef struct vpnp_adjust_in_t {
  bool adjust;

  bool adjust_auto;
  bool adjust_manual_roll;
  bool adjust_manual_pitch;

  bool adjust_reset;
  bool adjust_reset_pitch;
  bool adjust_reset_roll;

  float adjust_course_value;
  float adjust_pitch_value;
  float adjust_roll_value;
} vpnp_adjust_in_t;

typedef struct vpnp_state_t {
  bool valid;
  bool fail;

  bool master;
  bool link_valid;
  bool prep_done;

  bool ins_setup_flag;
  vpnp_adjust_in_t adjust_in;

  uint32_t sds01_p1;
  uint32_t sds02_p1;
  uint32_t sds03_p1;

  bool start_on_test;
  bool ground_control;
  bool setup;
  bool navigation;
  bool waiting;
  bool heating_bche;

  bool mode_adjust;
  bool low_accuracy;
  bool correct_svs;
  bool correct_sns;

  bool sns_valid;
  bool sns_fail;
  bool data_update;

  struct {
    bool valid;
    bool link;
    bool failure;
  } cdmk;

  struct {
    bool valid;
    bool failure;
    bool link;
    bool work;
  } ufos[2];

  bool mfi10_link;
  bool mfi12l_link;
  bool mfi12r_link;
} vpnp_state_t;

typedef enum fuel_tank_mode_e { TWO_TANKS = 0, FOUR_TANKS, FUEL_MODE_CNT } fuel_tank_mode_e;

typedef struct fuel_tank_mode_t {
  fuel_tank_mode_e value;
  bool valid;
} fuel_tank_mode_t;

//------------------ Переменные для логики комплекса ------------- zas
typedef enum complex_mode_e {
  PI_MODE_UNKNOWN = 0,
  PI_MODE_PODGOTOVKA = 1,
  PI_MODE_NAVIGATION = 2,
  PI_MODE_REZERV = 3
} complex_mode_e;

typedef enum takeoff_mode_e {
  ST_MODE_K_UNKNOWN = 0,
  ST_MODE_K_ZAPUSKU_ENGINE_NE_GOTOV,
  ST_MODE_K_VIRULIVANIU_NE_GOTOV,
  ST_MODE_K_VILETU_NE_GOTOV
} takeoff_mode_e;

typedef struct complex_t {
  complex_mode_e PI_MODE;
  takeoff_mode_e PI_START_MODE;
  bool PI_MOVE_DISABLE;
  int PI_PPLK;
  int PI_CTRL_VPNP1;
  int PI_CTRL_VPNP2;
  bool Setup;
  bool PI_Dev;
  bool PI_adjust;
  //------------------------
  bool PACK_owerwrite;
} complex_t;

//---------------------------------------------------------------- zas
typedef struct in_t {
  complex_t complex; // zas !!!!!!

  brightness_t brightness;
  me104_state_t me104_state;

  vpnp_state_t vpnp1;
  vpnp_state_t vpnp2;

  // агр

  param_t pitch; // тангаж
  param_t roll;  // крен
  param_t yaw;   // рыскание

  param_t flight_director_pitch; // Директорное управление по горизонтали [±1.0] (тангаж)
  param_t flight_director_roll; // Директорное управление по вертикали [±1.0] (крен)

  // шкала вертикальных скоростей
  param_t vy;       // Скорость вертикальная [±30]
  param_t vy_given; // Скорость вертикальная заданная ±30 м/с

  // шкала скоростей
  param_t device_speed;       // Приборная скорость Vп (поток воздуха)
  param_t true_air_speed;     // Истинная(воздушная) скорость Vа (true air speed)
  param_t ground_speed;       // Путевая скорость
  param_t given_speed;        // Заданная скорость
  param_t given_ctrl_speed;   // Заданная контрольная скорость
  param_t device_speed_trend; // Прогнозируемая приборная скорость(тренд)
  param_t min_allowed_speed;  // Vmax доп
  param_t max_allowed_speed;  // Vmin доп
  param_t M;                  // Число Маха
  param_t danger_speed_low_start; // Опасная скорость; нижняя граница нижнего ограничения
  param_t danger_speed_low_end; // Опасная скорость; верхняя граница нижнего ограничения
  param_t danger_speed_hi_start; // Опасная скорость; нижняя граница верхнего ограничения
  param_t danger_speed_hi_end; // Опасная скорость; верхняя граница верхнего ограничения

  // шкала высот
  param_t altitude;            // Абсолютная барометрическая высота
  param_t altitude_trend;      // Прогнозируемая баровысота(тренд)
  param_t altitude_given;      // Заданная высота
  param_t altitude_given_ctrl; // Заданная контрольная высота
  param_t altitude_deviation;  // Оклонение высоты
  param_t altitude_danger;     // Опасная высота
  param_t altitude_radio;      // Радиовысота

  pressure_type_t pressure_type; // Текущая коррекция баровысоты
  param_t pressure; // Давление(коррекция) Необходима синхронизация in out
  param_t pressure_correction_QFE; // Давление (коррекция QFE)
  param_t pressure_correction_QNH; // Давление (коррекция QNH)
  // навигационная шкала
  heading_display_t heading_display; // 0 - Магнитный, 1 - Истинный
  param_t heading_magnetic;          // Магнитный курс
  param_t heading_true;              // Истинный курс
  param_t heading_given;             // Заданный курс

  // param_t heading_angle_magnetic;    // Магнитный путевой угол (градусы)
  // param_t heading_angle;             // Истинный путевой угол (градусы)
  param_t track_angle_true;     // Истинный путевой угол (градусы)
  param_t track_angle_magnetic; // Магнитный путевой угол (градусы)
  param_t track_angle_given;    // Заданный путевой угол (градусы)
  // param_t heading_angle_given;       // Заданный путевой угол

  param_t magnetic_declination; // Магнитное склонение

  param_t radio_direction; // Курсовой угол радиостанции (АРК РСБН)
  param_t drift_angle;     // Угол сноса
  param_t wind_angle;      // Угол ветра
  param_t wind_direction;  // «НАВИГАЦИОННОЕ» направление ветра (КУДА ДУЕТ)
  param_t wind_speed;

  // параметры навигации
  params_t ppm_name; // Наименования исполняемого ППМ текущего полетного задания
  param_t ppm_time;         // Время пролета исполняемого ППМ (секунды)
  param_t ppm_altitude;     // Высота пролета исполняемого ППМ (метры)
  param_t ppm_azimuth;      // Азимут на исполняемый ППМ (градусы)
  param_t ppm_distance;     // Дальность до следующего ППМ (метры)
  param_t angular_velocity; // Угловая скорость (м/с) - пока что
  param_t cross_track_angl; // Линейное угловое уклонение (а на самом деле БУ) (градусы)
  param_t vert_velocity; // Вертикальная скорость (м/с)
  param_t turn_speed; // Скорость поворота (м/с) - !! не указано что за скорость и величина

  param_t current_correction_mode; // Индекс-указатель текущей системы коррекции (0 - нет коррекции, 1 - GPS, 2 - ГЛНС,
                                   // 3 - АВТО)
  param_t current_navigation_mode; // индекс-указатель текущего навигационного режима (0 - ПДГ, 1 - ВЗЛ, 2 - МРШ, 3 -
                                   // ПОС, 4 - ПНТ, 5 - ЗК)
  param_t cross_track_error;       // Линейное боковое уклонение (метры)

  // шкала ny/alpha
  param_t ny_min;
  param_t ny_max;
  param_t ny;

  param_t alpha_min;
  param_t alpha_max;
  param_t alpha;

  //
  param_t flaps_position;
  // param_t rudder_position;
  param_t aileron_left_position;
  param_t aileron_right_position;
  // param_t elevator_position;

  param_t aileron_trimmer_position; // На кадре НАВИГ, вкладка СУ, индицируется два значения (левый и правый), они ниже
  param_t aileron_trimmer_left_position;
  param_t aileron_trimmer_right_position;
  param_t rudder_trimmer_position;
  param_t elevator_trimmer_position;

  signal_t aileron_trimmer_n; // На кадре НАВИГ, вкладка СУ, индицируется два значения (левый и правый), они ниже
  signal_t aileron_trimmer_l_n;
  signal_t aileron_trimmer_r_n;
  signal_t rudder_trimmer_n;
  signal_t elevator_trimmer_n;

  signal_t air_sel_n;

  param_t engine_torque;
  param_t engine_power;
  // param_t engine_exhaust_temperature;
  // param_t engine_oil_pressure;
  // param_t engine_oil_temperature;
  // param_t engine_oil_valve_angle;

  // param_t fuel_pressure;
  param_t fuel_pressure_min;
  param_t fuel_pressure_max;
  // param_t fuel_tank_left;
  // param_t fuel_tank_right;
  param_t fuel_secondary_tank_left;
  param_t fuel_secondary_tank_right;
  fuel_tank_mode_t fuel_tank_mode;
  // param_t fuel_consumption;           // Понадобилось аж три потребления(ниже), непонятно какое это, без подписи

  param_t fuel_consumption_total;    // Израсходованно топлива в литрах
  param_t fuel_consumption_per_km;   // Расход топлива в литрах на км
  param_t fuel_consumption_per_hour; // Расход топлива в литрах в час

  param_t current_time_sec_sns;
  param_t current_time_min_sns;
  param_t current_time_hour_sns;
  param_t current_date_day_sns;
  param_t current_date_mon_sns;
  param_t current_date_year_sns;

  param_t flight_level;

  bool glonas_valid;
  bool gps_valid;

  paramd_t lat;
  paramd_t lon;

  //-- для кадра СИСТ
  // signal_t contr_pvd_right;           // КОНТР ПВД ПРАВЫЙ
  // signal_t contr_pvd_left;            // КОНТР ПВД ЛЕВЫЙ
  // signal_t stoper_hv_wheel;           // СТОПОР ХВ КОЛЕСА
  // signal_t picking_open;              // ОТБОР ОТКРЫТ
  // signal_t ppz_isnt_on;               // ППЗ НЕ ВКЛЮЧЕНА
  // signal_t auto_start_mode;           // АВТОМАТ РЕЖ ЗАПУСКА
  // signal_t emergency_ignition;         // АВАРИЙНОЕ ЗАЖИГАНИЕ
  // signal_t beta_mode;                 // БЕТА РЕЖИМ
  // signal_t rap_on;                    // РАП ВКЛЮЧЕН
  // signal_t pos_vv;                    // ПОС ВВ
  // signal_t pvd_heat;                  // ОБОГРЕВ ПВД
  // signal_t air_picking_heat;          // ОБОГР ВОЗД ЗАБОР
  // signal_t compressor_work;           // РАБ КОМПРЕССОР
  // signal_t ecn_work;                  // РАБОТА ЭЦН
  // signal_t apu_picking_open;          // ВСУ ОТБОР ОТКРЫТ
  // signal_t ttl_on;                    // ВКЛ TTL

  param_t air_temperature; // Температура воздуха

  //-- Исправности
  //---- модули
  param_health_t mfi12_1_valid;
  param_health_t mfi12_2_valid;
  param_health_t mfi10_valid;
  param_health_t vpnp_1_valid;
  param_health_t vpnp_2_valid;
  param_health_t ufos_1_valid;
  param_health_t ufos_2_valid;
  //---- линии связи
  param_health_t mfi12_1_vpnp_1_con_valid;
  param_health_t mfi10_vpnp_1_con_valid;
  param_health_t mfi10_vpnp_2_con_valid;
  param_health_t mfi12_2_vpnp_2_con_valid;
  param_health_t vpnp_1_ufos_1_con_valid;
  param_health_t vpnp_1_ufos_2_con_valid;
  param_health_t vpnp_1_cdmk_con_valid;
  param_health_t vpnp_2_ufos_1_con_valid;
  param_health_t vpnp_2_ufos_2_con_valid;
  param_health_t vpnp_2_cdmk_con_valid;

  // param_t battery_current;
  // param_t battery_voltage;
  // param_t generator_current;
  // param_t generator_voltage;
  param_t apu_voltage;

  // signal_t engine_start;              // ЗАПУСК ДВИГАТЕЛЯ
  // signal_t manual_mode;               // РУЧНОЙ РЕЖИМ
  // signal_t wow;
  // signal_t apu_start;                 // ВСУ ЗАПУСК
  // signal_t apu_work;                  // РАБОТА ВСУ
  // signal_t engine_fire;
  // signal_t apu_fire;
  // signal_t engine_failure;
  // signal_t generator_failure;

  param_t slide_position;
  param_t Nz_filter;
  param_t Ny_filter;
  param_t Vy_filter;
  param_t Alpha_filter;

  // param_t ramp_heading;
  // bool ramp_heading_use_magnetic;
  // param_t magnetic_declination;
  // param_t temperature;
  // date_t date;

  bool chassis;

  bool ex_controls;

  // PARTIZAN
  // ss1
  signal_t TVS2MS_on_AKB1;                                 // АКБ1 включен
  signal_t TVS2MS_on_AKB2;                                 // АКБ2 включен
  signal_t TVS2MS_on_GEN;                                  // ГЕН включен
  signal_t rap_on /*TVS2MS_on_SHRAP*/;                     // ШРАП включен
  signal_t TVS2MS_min_pressure_fuel;                       // Минимальное давление топлива
  signal_t TVS2MS_fire_hydrant_open;                       // Пожарный кран открыт
  signal_t TVS2MS_fire_hydrant_close;                      // Пожарный кран закрыт
  signal_t TVS2MS_fire;                                    // Пожар
  signal_t apu_work;                                       // ВСУ Работа
  signal_t apu_start;                                      // ВСУ Запуск
  signal_t auto_start_mode;                                // Автоматический режим запуска
  signal_t manual_mode;                                    // Ручной режим
  signal_t engine_fire /*TVS2MS_fire_engine_compartment*/; // Пожар в двигательном отсеке

  signal_t TVS2MS_neutral_position_of_Aileron_trim; // Нейтральное положение триммера элеронов
  signal_t engine_failure /*TVS2MS_engine_failure*/; // Отказ двигателя
  signal_t TVS2MS_neutral_position_of_RN_trim;       // Нейтральное положение триммера РН
  signal_t generator_failure /*TVS2MS_generator_failure*/; // Отказ генератора
  signal_t apu_fire;                                       // ВСУ Пожар
  signal_t TVS2MS_neutral_position_of_RV_trim;   // Нейтральное положение триммера РВ
  signal_t engine_start /*TVS2MS_engine_start*/; // Запуск двигателя
  signal_t ecn_work /*TVS2MS_ecn_work*/;         // Работа ЭНЦ
  signal_t air_picking_heat /*TVS2MS_heating_air_intake*/;   // Обогрев воздухозаборника
  signal_t pos_vv /*TVS2MS_pos_vv*/;                         // ПОС ВВ
  signal_t pvd_heating_left /*TVS2MS_heating_pvd*/;          // Обогрев ПВД
  signal_t compressor_work /*TVS2MS_work_compressor*/;       // Работа компрессора
  signal_t apu_picking_open /*TVS2MS_apu_pick_open*/;        // ВСУ отбор открыт
  signal_t ttl_on /*TVS2MS_on_TTL*/;                         // ВКЛ ТТЛ
  signal_t emergency_ignition /*TVS2MS_emergency_ignition*/; // Аварийное зажигание

  signal_t contr_pvd_right /*TVS2MS_pvd_control_right*/;  // Контроль ПВД правый
  signal_t contr_pvd_left /*TVS2MS_pvd_control_left*/;    // Контроль ПВД левый
  signal_t picking_open /*TVS2MS_pvd_pick_open*/;         // Отбор открыт
  signal_t stoper_hv_wheel /*TVS2MS_tail_wheel_stopper*/; // Стопор хвостового колеса
  signal_t TVS2MS_compressor_overheating;                 // Перегрев компрессора
  signal_t TVS2MS_apu_generator_failure;                  // ВСУ Отказ генератора
  signal_t TVS2MS_apu_failure;                            // ВСУ неисправно
  signal_t TVS2MS_apu_min_pressure_oil;                   // ВСУ Минимальное давление масла
  signal_t TVS2MS_engine_extinguishing;                   // Тушение двигатель
  signal_t TVS2MS_apu_extinguishing;                      // ВСУ Тушение
  signal_t pvd_heating_right;                             // Обогрев ПВД правый

  signal_t TVS2MS_door_isopen;             // Дверь открыта
  signal_t TVS2MS_fuel_filtr_foul;         // Топливный фильтр засорен
  signal_t TVS2MS_engine_min_pressure_oil; // Минимальное давление масла в двигателе
  signal_t TVS2MS_critical_fuel_balance_right_group_tanks; // Критический остаток топлива правой группы баков
  signal_t TVS2MS_critical_fuel_balance_left_group_tanks; // Критический остаток топлива левой группы баков
  signal_t TVS2MS_shavings_oil;               // Стружка в масле
  signal_t ppz_isnt_on /*TVS2MS_ppz_off*/;    // ППЗ не включена
  signal_t beta_mode /*TVS2MS_mode_betta*/;   // Бета режим
  signal_t wow_left;                          // Шасси обжато №1(ЛЕВ)
  signal_t wow_right;                         // Шасси обжато №2(ПРАВ)
  signal_t TVS2MS_pin_operation_off_AP_left;  // Кнопка оперативного ОТКЛ АП ЛЕВ
  signal_t TVS2MS_pin_operation_off_AP_right; // Кнопка оперативного ОТКЛ АП ПРАВ
  signal_t TVS2MS_switch_control_LA_NPU;      // Тумблер управление самолет/нпу
  signal_t TVS2MS_mode_RCU_auto;              // РСУ реж. АВТО
  signal_t TVS2MS_mode_RCU_Manual_Normal;     // РСУ реж. РУЧН Норма
  signal_t TVS2MS_mode_RCU_Manual_Pedals;     // РСУ реж. РУЧН. Педали

  param_health_t TVS2MS_valid_battery_current_1; // Достоверность параметра: Ток аккумулятора №1
  param_health_t TVS2MS_valid_battery_voltage_1; // Достоверность параметра: Напряжение аккумулятора №1
  param_health_t TVS2MS_valid_generator_current; // Достоверность параметра: Ток генератора
  param_health_t TVS2MS_valid_generator_voltage; // Достоверность параметра: Напряжение генератора
  param_health_t TVS2MS_valid_oil_pressure; // Достоверность параметра: Давление масла
  param_health_t TVS2MS_valid_fuel_pressure; // Достоверность параметра: Давление топлива
  param_health_t TVS2MS_valid_fuel_consumption; // Достоверрность параметра: Температура масла
  param_health_t
      TVS2MS_valid_fuel_remaining_left_group_tank; // Достоверность параметра: Остаток топлива в левой гр.баков
  param_health_t TVS2MS_valid_fuel_remaining_right_group_tank; // Достоверность параметра: Расход топлива
  param_health_t
      TVS2MS_valid_oil_temperature; // Достовеtank;//Достоверность параметра: Остаток топлива в правой гр.баков
  param_health_t TVS2MS_valid_screw_torque_value; // Достоверность параметра: Значение крутящего момента/шага винта
  param_health_t TVS2MS_valid_engine_n_value; // Достоверность параметра: Значение оборотов двигателя
  param_health_t TVS2MS_valid_engine_t_gas_out_value; // Достоверность параметра: Температура выходящих газов двигателя

  param_health_t TVS2MS_valid_oil_cooler_flap_position; // Достоверность параметра: Положение створки маслорадиатора
  param_health_t TVS2MS_valid_fuel_remaining_total_rezerv; // Достоверность параметра: Общий остаток топлива(резерв)
  param_health_t TVS2MS_valid_battery_current_2; // Достоверность параметра: Ток аккумулятора №2
  param_health_t TVS2MS_valid_battery_voltage_2; // Достоверность параметра: Напряжение аккумулятора №2
  param_health_t TVS2MS_valid_brake_pressure_system; // Достоверность параметра : Давление в тормозной системе
  param_health_t TVS2MS_valid_brake_pressure; // Достоверность параметра : Давление в тормозах
  param_health_t TVS2MS_valid_position_rudder; // Достоверность параметра : Положение руля направления
  param_health_t TVS2MS_valid_position_elevator; // Достоверность параметра : Положение руля высоты
  param_health_t TVS2MS_valid_position_aileron; // Достоверность параметра : Положение элеронов
  param_health_t TVS2MS_valid_position_upper_flap_lb; // Достоверность параметра : Положение закрылков верхних ЛБ
  param_health_t TVS2MS_valid_position_down_flap_lb; // Достоверность параметра : Положение закрылков нижних   ЛБ
  param_health_t TVS2MS_valid_position_slat; // Достоверность параметра : Положение предкрылков
  param_health_t TVS2MS_valid_position_upper_flap_pb; // Достоверность параметра : Положение закрылков верхних ПБ
  param_health_t TVS2MS_valid_position_down_flap_pb; // Достоверность параметра : Положение закрылков нижних   ПБ

  param_t battery_current_1; // INT16//       //1         Ток аккумулятора №1
  param_t battery_voltage_1; // UINT16//      //0.1         Напряжение аккумулятора №1
  param_t battery_current_2; // INT16//       //1         Ток аккумулятора №2
  param_t battery_voltage_2; // UINT16//      //0.1         Напряжение аккумулятора №2
  param_t generator_current /*TVS2MS_generator_current*/; // INT16//       //1         Ток генератора
  param_t generator_voltage; // UINT16//      //0.1         Напряжение генератора
  param_t engine_oil_pressure /*TVS2MS_oil_pressure*/;  // UINT16//           //1         Давление масла
  param_t fuel_pressure /*TVS2MS_fuel_pressure*/;       // UINT16//          //1         Давление топлива
  param_t fuel_consumption /*TVS2MS_fuel_consumption*/; // UINT16//       //1         Расход топлива
  param_t engine_oil_temperature /*TVS2MS_oil_temperature*/; // INT16//         //1         Температура масла
  param_t fuel_tank_left /*TVS2MS_fuel_remaining_left_group_tank*/; // 1         //UINT16//Остаток топлива в левой
                                                                    // группе баков
  param_t fuel_tank_right /*TVS2MS_fuel_remaining_right_group_tank*/; // 1         //UINT16//Остаток топлива в правом
                                                                      // группе баков
  param_t TVS2MS_screw_torque_value; // 1         //UINT16//Значение крутящего момента/шага винта
  param_t TVS2MS_engine_n_value; // 1         //UINT16//Значение оборотов двигателя
  param_t engine_exhaust_temperature /*TVS2MS_engine_t_gas_out_value*/; // 1         //UINT16//Температура выходящих
                                                                        // газов двигателя
  param_t TVS2MS_r3; //         //UINT16//Зарезервированно
  param_t engine_oil_valve_angle /*TVS2MS_oil_cooler_flap_position*/; // 1         //UINT16//Положение створки
                                                                      // маслорадиатора
  param_t TVS2MS_battery_current_2;     // 1         //INT16//Ток аккумулятора №2
  param_t TVS2MS_battery_voltage_2;     // 0.1         //UINT16//Напряжение аккумулятора №2
  param_t TVS2MS_brake_pressure_system; // 0.1         //UINT16//Давление в тормозной системе
  param_t TVS2MS_brake_pressure;        // 0.1         //UINT16//Давление в в тормозах
  param_t rudder_position /*TVS2MS_position_rudder*/; // 0.01         //INT16//Положение руля направления
  param_t elevator_position /*TVS2MS_position_elevator*/; // 0.01         //INT16//Положение руля высоты
  param_t TVS2MS_position_aileron;                        // 0.01         //INT16//Положение элеронов
  param_t TVS2MS_position_upper_flap_lb; // 0.01         //UINT16//Положение закрылков верхней секции крыла ЛБ
  param_t TVS2MS_position_down_flap_lb; // 0.01         //UINT16//Положение закрылков нижней секции крыла  ЛБ
  param_t TVS2MS_position_slat; // 0.01         //UINT16//Положение предкрылков
  param_t TVS2MS_position_upper_flap_pb; // 0.01         //UINT16//Положение закрылков верхней секции крыла ПБ
  param_t TVS2MS_position_down_flap_pb; // 0.01         //UINT16//Положение закрылков нижней секции крыла  ПБ

} in_t;

void in_step(const in_t *in);

#ifdef __cplusplus
}
#endif
