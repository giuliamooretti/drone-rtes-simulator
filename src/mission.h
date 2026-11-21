#ifndef MISSION_H
#define MISSION_H

#include "shared_state.h"

typedef struct {
    setpoint_t *setpoint;
    drone_state_t *state;   // to know when the drone has actually reached the current waypoint
} mission_args_t;

void *mission_thread(void *arg);

#endif /* MISSION_H */
