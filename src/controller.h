#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <mujoco/mujoco.h>
#include "shared_state.h"

typedef struct {
    mjModel *model;
    mjData *data;
    drone_state_t *state;      
    setpoint_t *setpoint;  
    safety_flag_t *safety;    
    control_health_t *health;    
} controller_args_t;


void *controller_thread(void *arg);

#endif /* CONTROLLER_H */
