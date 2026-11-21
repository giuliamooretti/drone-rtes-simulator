#include <math.h>
#include "state_estimator.h"
#include "common.h"
#include "utils.h"

// Converts a quaternion (w,x,y,z) to Euler angles (ZYX convention)
static void quat_to_euler(const double q[4], double *roll, double *pitch, double *yaw) {
    double w = q[0], x = q[1], y = q[2], z = q[3];

    // atan2 instead of atan so we don't lose the sign information (evaluates num and den separately)
    *roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));

    double sinp = 2.0 * (w * y - z * x);
    // clamp for safety
    if (sinp > 1.0) sinp = 1.0;
    if (sinp < -1.0) sinp = -1.0;
    *pitch = asin(sinp);

    *yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void *estimator_thread(void *arg) {
    estimator_args_t *args = (estimator_args_t *)arg;
    const long period_ns = 1000000000L / ESTIMATOR_HZ;

    int adr_gyro = args->model->sensor_adr[mj_name2id(args->model, mjOBJ_SENSOR, "body_gyro")];
    int adr_quat = args->model->sensor_adr[mj_name2id(args->model, mjOBJ_SENSOR, "body_quat")];

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    while (g_running) {
        double gyro[3];
        double quat[4];
        double pos[3];
        double vel[3];

        pthread_mutex_lock(&sim_mutex);
        gyro[0] = args->data->sensordata[adr_gyro + 0];
        gyro[1] = args->data->sensordata[adr_gyro + 1];
        gyro[2] = args->data->sensordata[adr_gyro + 2];
        
        quat[0] = args->data->sensordata[adr_quat + 0];
        quat[1] = args->data->sensordata[adr_quat + 1];
        quat[2] = args->data->sensordata[adr_quat + 2];
        quat[3] = args->data->sensordata[adr_quat + 3];

        // treated as if it were an ideal, noise-free altimeter/GPS -> a real one would need a filter
        pos[0] = args->data->qpos[0];
        pos[1] = args->data->qpos[1];
        pos[2] = args->data->qpos[2];

        // qvel[0..2] is the linear velocity in world frame, read as if from an ideal velocity sensor
        vel[0] = args->data->qvel[0];
        vel[1] = args->data->qvel[1];
        vel[2] = args->data->qvel[2];
        pthread_mutex_unlock(&sim_mutex);   // lock only for the shared access, not for the computation

        double roll, pitch, yaw;
        quat_to_euler(quat, &roll, &pitch, &yaw);

        drone_state_write_attitude(args->state, roll, pitch, yaw, gyro[0], gyro[1], gyro[2]);

        // Simulated GPS loss
        fault_flags_t faults;
        fault_get(args->faults, &faults);
        if (!faults.gps_lost) {
            drone_state_write_position(args->state, pos[0], pos[1], pos[2]);
            drone_state_write_velocity(args->state, vel[0], vel[1], vel[2]);
        }

        timespec_add_ns(&next_tick, period_ns);
        sleep_until(&next_tick);
    }
    return NULL;
}
