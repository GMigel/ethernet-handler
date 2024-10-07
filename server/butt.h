
#pragma once

#include "controls.h"
#include "state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct butt_hdl butt_hdl;

butt_hdl *
butt_init(const state_t *state);
void butt_fini(butt_hdl *);
void butt_step(butt_hdl *,
               state_t *state,
               controls_t *controls);

#ifdef __cplusplus
}
#endif
