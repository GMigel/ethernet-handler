#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef enum butt_enum_t {
    BEMPTY = 0,
    TOP1,
    TOP2,
    TOP3,
    TOP4,
    TOP5,
    TOP6,
    TOP7,
    TOP8,
    BOTTOM1,
    BOTTOM2,
    BOTTOM3,
    BOTTOM4,
    BOTTOM5,
    BOTTOM6,
    BOTTOM7,
    BOTTOM8,
    LEFT1,
    LEFT2,
    LEFT3,
    LEFT4,
    LEFT5,
    LEFT6,
    RIGHT1,
    RIGHT2,
    RIGHT3,
    RIGHT4,
    RIGHT5,
    RIGHT6,
    BUTT_M0,
    BUTT_M1,
    BUTTONS_COUNT = BUTT_M1 + 1,
} butt_enum_t;

typedef enum wheel_enum_t {
    WEMPTY = 0,
    LEFT_TOP,
    LEFT_BOTTOM,
    RIGHT_TOP,
    RIGHT_BOTTOM,
    WHEELS_COUNT,
} wheel_enum_t;


#ifdef __cplusplus
}
#endif
