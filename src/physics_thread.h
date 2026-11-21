#ifndef PHYSICS_THREAD_H
#define PHYSICS_THREAD_H

#include <mujoco/mujoco.h>
#include "shared_state.h"

typedef struct {
    mjModel *model;
    mjData *data;
    safety_flag_t *safety;   // if emergency is active, zero ctrl[] before stepping
    fault_flags_t *faults;   // failed rotors (ctrl forced to 0) and wind gust
} physics_args_t;

void *physics_thread(void *arg);

#endif /* PHYSICS_THREAD_H */
