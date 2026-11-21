// Definition of the shared state for the whole system
#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <pthread.h>
#include <stdbool.h>
#include <time.h>
#include <signal.h>

// Drone state estimate: 1 writer, N readers -> pthread_rwlock_t
typedef struct {
    pthread_rwlock_t lock;
    double roll, pitch, yaw;
    double roll_rate, pitch_rate, yaw_rate;

    double x, y, z;

    double vx, vy, vz;  // linear velocity
} drone_state_t;
// pthread_rwlock_rdlock() lets multiple threads hold the lock for reading at the same time
// only when someone calls pthread_rwlock_wrlock() (writer) does the lock become exclusive, blocking both other writers and readers


// Setpoint is written by the mission thread, read by the controller thread -> producer/consumer pattern
typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t cond;   // signals the arrival of a new target

    double target_altitude;
    double target_x, target_y; // horizontal position to hold
    double target_roll, target_pitch, target_yaw;
    bool has_new_setpoint;
} setpoint_t;

// Shared safety flag, written by the watchdog, read by the controller and physics_thread
typedef struct {
    pthread_mutex_t mutex;
    bool emergency_active;
    char reason[64];
} safety_flag_t;

// Telemetry/battery/log data, periodically written by monitor -> this mutex is the one used for the priority inversion demo
typedef struct {
    pthread_mutex_t mutex;
    double battery_level;
    long log_entries_written;
} monitor_data_t;

// Health status of the control loop
typedef struct {
    pthread_mutex_t mutex;
    struct timespec last_cycle_end;
    int consecutive_deadline_misses;
} control_health_t;

// Faults injectable from the keyboard
typedef struct {
    pthread_mutex_t mutex;
    bool rotor_failed[4];
    bool wind_gust_active;
    bool gps_lost;
} fault_flags_t;

// Mutex protecting access to mjData
extern pthread_mutex_t sim_mutex;

// Termination flag: main clears it when the viewer window is closed
extern volatile sig_atomic_t g_running;

void shared_state_init(drone_state_t *state, setpoint_t *setpoint, safety_flag_t *safety, monitor_data_t *monitor, control_health_t *health, fault_flags_t *faults);
void shared_state_destroy(drone_state_t *state, setpoint_t *setpoint, safety_flag_t *safety, monitor_data_t *monitor, control_health_t *health, fault_flags_t *faults);

// Helpers for accessing the estimated state (rwlock)
void drone_state_read(drone_state_t *state, drone_state_t *out_copy);
void drone_state_write_attitude(drone_state_t *state, double roll, double pitch, double yaw, double roll_rate, double pitch_rate, double yaw_rate);
void drone_state_write_position(drone_state_t *state, double x, double y, double z);
void drone_state_write_velocity(drone_state_t *state, double vx, double vy, double vz);

// Setpoint helpers (mutex + cond var)
void setpoint_publish(setpoint_t *sp, double altitude, double x, double y, double roll, double pitch, double yaw);
void setpoint_wait_and_get(setpoint_t *sp, double *out_altitude, double *out_x, double *out_y, double *out_roll, double *out_pitch, double *out_yaw);

// Safety flag helpers
void safety_trigger(safety_flag_t *flag, const char *reason);
void safety_clear(safety_flag_t *flag);
bool safety_is_active(safety_flag_t *flag);

// monitor_data_t helper (read for the on-screen overlay)
double monitor_get_battery(monitor_data_t *monitor);

// Control health helpers (used by controller and watchdog)
void control_health_report_cycle(control_health_t *health);
int  control_health_check_and_reset(control_health_t *health, double deadline_ms, int *out_consecutive_misses);

// Injectable fault helpers (used by main, physics_thread, state_estimator)
void fault_set_rotor(fault_flags_t *faults, int rotor_index, bool failed);
void fault_set_wind(fault_flags_t *faults, bool active);
void fault_set_gps_lost(fault_flags_t *faults, bool lost);
void fault_clear_all(fault_flags_t *faults);
void fault_get(fault_flags_t *faults, fault_flags_t *out_copy);

#endif /* SHARED_STATE_H */
