#include "controller.h"
#include "common.h"
#include "utils.h"

// Rotor geometry and yaw reaction coefficient
#define ARM_Y      0.18
#define ARM_X      0.14
#define YAW_COEFF  0.0201

// PID gains
#define KP_Z    8.0
#define KD_Z    4.0
#define KP_ATT  0.6
#define KD_ATT  0.08
#define KP_YAW  0.3
#define KD_YAW  0.05

#define KP_POS  0.03
#define KD_POS  0.10
#define MAX_POS_HOLD_TILT_RAD 0.35  // about 20 degrees: enough to counter the wind

#define HOVER_THRUST_FF 13.0

static double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void *controller_thread(void *arg) {
    controller_args_t *args = (controller_args_t *)arg;
    const long period_ns = 1000000000L / CONTROLLER_HZ;
    const double dt = 1.0 / CONTROLLER_HZ;

    int act_id[NUM_ROTORS];
    act_id[0] = mj_name2id(args->model, mjOBJ_ACTUATOR, "thrust1");
    act_id[1] = mj_name2id(args->model, mjOBJ_ACTUATOR, "thrust2");
    act_id[2] = mj_name2id(args->model, mjOBJ_ACTUATOR, "thrust3");
    act_id[3] = mj_name2id(args->model, mjOBJ_ACTUATOR, "thrust4");
    // ctrl_min/ctrl_max are read from the model
    double ctrl_min = args->model->actuator_ctrlrange[2 * act_id[0]];
    double ctrl_max = args->model->actuator_ctrlrange[2 * act_id[0] + 1];

    // the controller needs to remember the previous cycle's error to compute how fast it's changing
    double prev_altitude_error = 0.0;

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    while (g_running) {
        drone_state_t st;
        drone_state_read(args->state, &st);

        double target_altitude, target_x, target_y, target_roll, target_pitch, target_yaw;
        setpoint_wait_and_get(args->setpoint, &target_altitude, &target_x, &target_y, &target_roll, &target_pitch, &target_yaw);

        // Altitude -> PD on the error
        // derivative approximated by finite differences (dt is fixed since this is a periodic loop)
        double altitude_error = target_altitude - st.z;
        double altitude_error_deriv = (altitude_error - prev_altitude_error) / dt;
        prev_altitude_error = altitude_error;
        double total_thrust = HOVER_THRUST_FF + KP_Z * altitude_error + KD_Z * altitude_error_deriv;

        // Position -> the horizontal position error becomes a small correction added to the attitude target
        // Pitching forward moves the drone along x, rolling sideways moves it along y; velocity (from the estimator) damps the correction instead of letting it oscillate
        double x_error = target_x - st.x;
        double y_error = target_y - st.y;
        double pos_pitch_cmd = clampd(KP_POS * x_error - KD_POS * st.vx, -MAX_POS_HOLD_TILT_RAD, MAX_POS_HOLD_TILT_RAD);
        double pos_roll_cmd = clampd(-KP_POS * y_error + KD_POS * st.vy, -MAX_POS_HOLD_TILT_RAD, MAX_POS_HOLD_TILT_RAD);

        // Attitude -> PD on the error + direct damping on the rate measured by the gyroscope
        double roll_error = (target_roll + pos_roll_cmd) - st.roll;
        double pitch_error = (target_pitch + pos_pitch_cmd) - st.pitch;
        double yaw_error = target_yaw - st.yaw;

        double torque_roll = KP_ATT * roll_error - KD_ATT * st.roll_rate;
        double torque_pitch = KP_ATT * pitch_error - KD_ATT * st.pitch_rate;
        double torque_yaw = KP_YAW * yaw_error - KD_YAW * st.yaw_rate;

        // Mixer: inverts the [thrust; roll; pitch; yaw] matrix -> thrust for the 4 rotors
        double f[NUM_ROTORS];
        f[0] = total_thrust / 4.0 - torque_roll / (4.0 * ARM_Y) + torque_pitch / (4.0 * ARM_X) - torque_yaw / (4.0 * YAW_COEFF);
        f[1] = total_thrust / 4.0 + torque_roll / (4.0 * ARM_Y) + torque_pitch / (4.0 * ARM_X) + torque_yaw / (4.0 * YAW_COEFF);
        f[2] = total_thrust / 4.0 + torque_roll / (4.0 * ARM_Y) - torque_pitch / (4.0 * ARM_X) - torque_yaw / (4.0 * YAW_COEFF);
        f[3] = total_thrust / 4.0 - torque_roll / (4.0 * ARM_Y) - torque_pitch / (4.0 * ARM_X) + torque_yaw / (4.0 * YAW_COEFF);

        // Note: no need to check the emergency flag here. physics_thread zeroes ctrl[] on every step while it's active, so it always has the last word
        pthread_mutex_lock(&sim_mutex);
        for (int i = 0; i < NUM_ROTORS; i++) {
            args->data->ctrl[act_id[i]] = clampd(f[i], ctrl_min, ctrl_max);     // the sim_mutex prevents this write from interleaving with physics_thread's mj_step(), which reads ctrl[] to compute the next step
        }
        pthread_mutex_unlock(&sim_mutex);

        control_health_report_cycle(args->health);

        timespec_add_ns(&next_tick, period_ns);
        sleep_until(&next_tick);
    }
    return NULL;
}
