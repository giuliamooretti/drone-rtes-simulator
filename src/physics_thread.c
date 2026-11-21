#include "physics_thread.h"
#include "common.h"
#include "utils.h"

void *physics_thread(void *arg) {
    physics_args_t *args = (physics_args_t *)arg;
    const long period_ns = 1000000000L / PHYSICS_HZ;    //  1 second expressed in nanoseconds + L (treat as long)

    int act_id[NUM_ROTORS];
    act_id[0] = mj_name2id(args->model, mjOBJ_ACTUATOR, "thrust1");     // mj_name2id() looks up a string name ("thrust1") in MuJoCo's model tables and returns an integer index
    act_id[1] = mj_name2id(args->model, mjOBJ_ACTUATOR, "thrust2");
    act_id[2] = mj_name2id(args->model, mjOBJ_ACTUATOR, "thrust3");
    act_id[3] = mj_name2id(args->model, mjOBJ_ACTUATOR, "thrust4");
    int body_id = mj_name2id(args->model, mjOBJ_BODY, "x2");

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick); // Reads the system clock at this instant and stores it in next_tick

    while (g_running) { // defined in shared_state.h
        fault_flags_t faults;
        fault_get(args->faults, &faults);

        pthread_mutex_lock(&sim_mutex);
        if (safety_is_active(args->safety)) {
            // Failsafe: cuts the motors
            for (int i = 0; i < args->model->nu; i++) {     // args->model->nu: the actual number of actuators in the model, read from MuJoCo
                args->data->ctrl[i] = 0.0;
            }
        } else {
            for (int i = 0; i < NUM_ROTORS; i++) {
                if (faults.rotor_failed[i]) {
                    // overrides the controller's command for that specific motor, regardless of what the controller writes (which knows nothing about the failure)
                    args->data->ctrl[act_id[i]] = 0.0;
                }
            }
        }

        // Wind gust
        if (faults.wind_gust_active) {
            args->data->xfrc_applied[6 * body_id + 0] = WIND_FORCE_N;   // xfrc_applied is a MuJoCo mechanism different from ctrl[]: it's a vector of external forces applied directly to bodies, independent of actuators -> "the wind pushes the drone from outside", not "a motor produces more thrust"
        } else {
            args->data->xfrc_applied[6 * body_id + 0] = 0.0;
        }

        mj_step(args->model, args->data);   // advances the physics simulation by one time step
        pthread_mutex_unlock(&sim_mutex);

        timespec_add_ns(&next_tick, period_ns);
        sleep_until(&next_tick);
    }
    return NULL;
}
