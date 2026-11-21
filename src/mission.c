#include <math.h>
#include <stdbool.h>
#include "mission.h"
#include "common.h"
#include "utils.h"

#define CRUISE_ALTITUDE_M 1.0
#define WAYPOINT_TOLERANCE_M 0.005   // distance within which the waypoint is considered reached
#define WAYPOINT_SPEED_TOLERANCE_MS 0.01 // it must also have stopped, not just passed nearby while still sliding
#define WAYPOINT_TIMEOUT_S   12.0    // moves on to the next waypoint anyway if it never arrives (-> strong wind)

// Square path
static const double waypoints_x[] = {0.0, 1.0, 1.0, 0.0};
static const double waypoints_y[] = {0.0, 0.0, 1.0, 1.0};
#define NUM_WAYPOINTS (sizeof(waypoints_x) / sizeof(waypoints_x[0]))    // array size / size of 1 element = number of elements

void *mission_thread(void *arg) {
    mission_args_t *args = (mission_args_t *)arg;
    const long period_ns = 1000000000L / MISSION_HZ;
    const long timeout_ticks = (long)(WAYPOINT_TIMEOUT_S * MISSION_HZ);

    struct timespec next_tick;  // struct timespec is a type defined in <time.h> representing a point in time with nanosecond precision
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    size_t wp = 0;
    long ticks_on_this_wp = 0;
    while (g_running) {
        // Advances to the next waypoint only once the drone has actually reached the current one (or after a timeout, so we don't get stuck forever if it never arrives)
        drone_state_t st;
        drone_state_read(args->state, &st);
        double dx = waypoints_x[wp] - st.x;
        double dy = waypoints_y[wp] - st.y;
        double dist = sqrt(dx * dx + dy * dy);
        double speed = sqrt(st.vx * st.vx + st.vy * st.vy);
        bool arrived = (dist < WAYPOINT_TOLERANCE_M && speed < WAYPOINT_SPEED_TOLERANCE_MS);

        if (ticks_on_this_wp > 0 && (arrived || ticks_on_this_wp >= timeout_ticks)) {
            wp = (wp + 1) % NUM_WAYPOINTS;
            ticks_on_this_wp = 0;
        }

        setpoint_publish(args->setpoint, CRUISE_ALTITUDE_M, waypoints_x[wp], waypoints_y[wp], 0.0, 0.0, 0.0);   // the last parameters are always 0.0 because mission decides where the drone should go in space, not how much it should tilt to get there -> that's the controller's job)
        ticks_on_this_wp++;

        timespec_add_ns(&next_tick, period_ns);
        sleep_until(&next_tick);
    }
    return NULL;
}
