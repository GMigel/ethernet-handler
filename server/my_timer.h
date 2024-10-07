#pragma once

typedef enum my_timer_mode_e{
    MY_TIMER_STANDBY,
    MY_TIMER_RUN,
    MY_TIMER_STOP,
    MY_TIMER
}my_timer_mode_e;

typedef struct state_t state_t;

void my_timer_init(state_t* state);
void my_timer_start(state_t* state);
void my_timer_on_step(state_t* state);
void my_timer_stop(state_t* state);
void my_timer_reset(state_t* state);