#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <mujoco/mujoco.h>
#include "shared_state.h"

typedef struct {
    mjModel *model;
    mjData *data;
    drone_state_t *state;   
    fault_flags_t *faults;
} estimator_args_t;

void *estimator_thread(void *arg);

#endif /* STATE_ESTIMATOR_H */
