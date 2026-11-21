#ifndef MONITOR_H
#define MONITOR_H

#include "shared_state.h"

typedef struct {
    monitor_data_t *monitor;
    drone_state_t *state;
} monitor_args_t;

void *monitor_thread(void *arg);

#endif /* MONITOR_H */
