#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "shared_state.h"

typedef struct {
    control_health_t *health;   // controller deadline misses
    drone_state_t *state;       // excessive tilt
    safety_flag_t *safety;      // safety_trigger() when needed
    monitor_data_t *monitor;    // battery (-> priority inversion)
} watchdog_args_t;

void *watchdog_thread(void *arg);

#endif /* WATCHDOG_H */
