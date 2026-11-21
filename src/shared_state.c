// Shared state for the whole system
#include <string.h>
#include "shared_state.h"
#include "common.h"

pthread_mutex_t sim_mutex = PTHREAD_MUTEX_INITIALIZER;  // statically initializes the mutex
volatile sig_atomic_t g_running = 1;    // sig_atomic_t is an integer type that C guarantees is readable/writable in a single indivisible atomic operation

void shared_state_init(drone_state_t *state, setpoint_t *setpoint, safety_flag_t *safety, monitor_data_t *monitor, control_health_t *health, fault_flags_t *faults) {
    memset(state, 0, sizeof(*state));
    pthread_rwlock_init(&state->lock, NULL);

    memset(setpoint, 0, sizeof(*setpoint));
    pthread_mutex_init(&setpoint->mutex, NULL);
    pthread_cond_init(&setpoint->cond, NULL);

    memset(safety, 0, sizeof(*safety));
    pthread_mutex_init(&safety->mutex, NULL);

    memset(monitor, 0, sizeof(*monitor));
    monitor->battery_level = 100.0;
    
    pthread_mutexattr_t monitor_mutex_attr;
    pthread_mutexattr_init(&monitor_mutex_attr);
    if (USE_PRIORITY_INHERITANCE) {
        pthread_mutexattr_setprotocol(&monitor_mutex_attr, PTHREAD_PRIO_INHERIT);
    }
    pthread_mutex_init(&monitor->mutex, &monitor_mutex_attr);
    pthread_mutexattr_destroy(&monitor_mutex_attr);
    

    memset(health, 0, sizeof(*health));
    pthread_mutex_init(&health->mutex, NULL);
    clock_gettime(CLOCK_MONOTONIC, &health->last_cycle_end);

    memset(faults, 0, sizeof(*faults));
    pthread_mutex_init(&faults->mutex, NULL);
}

void shared_state_destroy(drone_state_t *state, setpoint_t *setpoint, safety_flag_t *safety, monitor_data_t *monitor, control_health_t *health, fault_flags_t *faults) {
    pthread_rwlock_destroy(&state->lock);
    pthread_mutex_destroy(&setpoint->mutex);
    pthread_cond_destroy(&setpoint->cond);
    pthread_mutex_destroy(&safety->mutex);
    pthread_mutex_destroy(&monitor->mutex);
    pthread_mutex_destroy(&health->mutex);
    pthread_mutex_destroy(&faults->mutex);
}

void drone_state_read(drone_state_t *state, drone_state_t *out_copy) {
    pthread_rwlock_rdlock(&state->lock);
    *out_copy = *state;     // Note that this copies the whole struct in one shot, not field by field. This keeps the state consistent.
    pthread_rwlock_unlock(&state->lock);
}


void drone_state_write_attitude(drone_state_t *state, double roll, double pitch, double yaw, double roll_rate, double pitch_rate, double yaw_rate) {
    pthread_rwlock_wrlock(&state->lock);
    state->roll = roll;
    state->pitch = pitch;
    state->yaw = yaw;
    state->roll_rate = roll_rate;
    state->pitch_rate = pitch_rate;
    state->yaw_rate = yaw_rate;
    pthread_rwlock_unlock(&state->lock);
}

void drone_state_write_position(drone_state_t *state, double x, double y, double z) {
    pthread_rwlock_wrlock(&state->lock);
    state->x = x;
    state->y = y;
    state->z = z;
    pthread_rwlock_unlock(&state->lock);
}

void drone_state_write_velocity(drone_state_t *state, double vx, double vy, double vz) {
    pthread_rwlock_wrlock(&state->lock);
    state->vx = vx;
    state->vy = vy;
    state->vz = vz;
    pthread_rwlock_unlock(&state->lock);
}

void setpoint_publish(setpoint_t *sp, double altitude, double x, double y, double roll, double pitch, double yaw) {
    pthread_mutex_lock(&sp->mutex);
    sp->target_altitude = altitude;
    sp->target_x = x;
    sp->target_y = y;
    sp->target_roll = roll;
    sp->target_pitch = pitch;
    sp->target_yaw = yaw;
    sp->has_new_setpoint = true;
    pthread_cond_signal(&sp->cond);
    pthread_mutex_unlock(&sp->mutex);
}

void setpoint_wait_and_get(setpoint_t *sp, double *out_altitude, double *out_x, double *out_y, double *out_roll, double *out_pitch, double *out_yaw) {
    pthread_mutex_lock(&sp->mutex);
    
    while (!sp->has_new_setpoint) {
        pthread_cond_wait(&sp->cond, &sp->mutex);
    }

    *out_altitude = sp->target_altitude;
    *out_x = sp->target_x;
    *out_y = sp->target_y;
    *out_roll = sp->target_roll;
    *out_pitch = sp->target_pitch;
    *out_yaw = sp->target_yaw;
    
    pthread_mutex_unlock(&sp->mutex);
}

void safety_trigger(safety_flag_t *flag, const char *reason) {
    pthread_mutex_lock(&flag->mutex);

    flag->emergency_active = true;
    strncpy(flag->reason, reason, sizeof(flag->reason) - 1);
    flag->reason[sizeof(flag->reason) - 1] = '\0';

    pthread_mutex_unlock(&flag->mutex);
}

void safety_clear(safety_flag_t *flag) {
    pthread_mutex_lock(&flag->mutex);
    flag->emergency_active = false;
    flag->reason[0] = '\0';
    pthread_mutex_unlock(&flag->mutex);
}

bool safety_is_active(safety_flag_t *flag) {
    pthread_mutex_lock(&flag->mutex);
    bool active = flag->emergency_active;
    pthread_mutex_unlock(&flag->mutex);
    return active;
}

double monitor_get_battery(monitor_data_t *monitor) {
    pthread_mutex_lock(&monitor->mutex);
    double battery = monitor->battery_level;
    pthread_mutex_unlock(&monitor->mutex);
    return battery;
}

void control_health_report_cycle(control_health_t *health) {
    pthread_mutex_lock(&health->mutex);
    clock_gettime(CLOCK_MONOTONIC, &health->last_cycle_end);
    health->consecutive_deadline_misses = 0;
    pthread_mutex_unlock(&health->mutex);
}

int control_health_check_and_reset(control_health_t *health, double deadline_ms, int *out_consecutive_misses) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    pthread_mutex_lock(&health->mutex);

    // tv_sec in ms + tv_nsec in ms, to get the elapsed time as a single number
    double elapsed_ms = (now.tv_sec - health->last_cycle_end.tv_sec) * 1000.0 + (now.tv_nsec - health->last_cycle_end.tv_nsec) / 1e6;
    int missed = elapsed_ms > deadline_ms;
    if (missed) {
        health->consecutive_deadline_misses++;
    }
    *out_consecutive_misses = health->consecutive_deadline_misses;
    
    pthread_mutex_unlock(&health->mutex);

    return missed;
}

void fault_set_rotor(fault_flags_t *faults, int rotor_index, bool failed) {
    pthread_mutex_lock(&faults->mutex);
    faults->rotor_failed[rotor_index] = failed;
    pthread_mutex_unlock(&faults->mutex);
}

void fault_set_wind(fault_flags_t *faults, bool active) {
    pthread_mutex_lock(&faults->mutex);
    faults->wind_gust_active = active;
    pthread_mutex_unlock(&faults->mutex);
}

void fault_set_gps_lost(fault_flags_t *faults, bool lost) {
    pthread_mutex_lock(&faults->mutex);
    faults->gps_lost = lost;
    pthread_mutex_unlock(&faults->mutex);
}

void fault_clear_all(fault_flags_t *faults) {
    pthread_mutex_lock(&faults->mutex);
    for (int i = 0; i < 4; i++) {
        faults->rotor_failed[i] = false;
    }
    faults->wind_gust_active = false;
    faults->gps_lost = false;
    pthread_mutex_unlock(&faults->mutex);
}

void fault_get(fault_flags_t *faults, fault_flags_t *out_copy) {
    pthread_mutex_lock(&faults->mutex);
    for (int i = 0; i < 4; i++) {
        out_copy->rotor_failed[i] = faults->rotor_failed[i];
    }
    out_copy->wind_gust_active = faults->wind_gust_active;
    out_copy->gps_lost = faults->gps_lost;
    pthread_mutex_unlock(&faults->mutex);
}
