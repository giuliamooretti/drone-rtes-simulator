#include <stdio.h>
#include <math.h>
#include "watchdog.h"
#include "common.h"
#include "utils.h"

void *watchdog_thread(void *arg) {
    watchdog_args_t *args = (watchdog_args_t *)arg;
    const long period_ns = 1000000000L / WATCHDOG_HZ;

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    while (g_running) {
        int misses;
        control_health_check_and_reset(args->health, CONTROL_DEADLINE_MS, &misses);
        if (misses >= MAX_CONSECUTIVE_DEADLINE_MISSES) {
            safety_trigger(args->safety, "control: too many consecutive deadline misses");
        }

        drone_state_t st;
        drone_state_read(args->state, &st);
        if (fabs(st.roll) > MAX_TILT_RAD || fabs(st.pitch) > MAX_TILT_RAD) {    // fabs = absolute value for floating point numbers
            safety_trigger(args->safety, "attitude: excessive tilt");
        }

        // With USE_PRIORITY_INHERITANCE=0, cpu_load_thread can inflate this wait well beyond the base duration of the critical section
        struct timespec wait_start, wait_end;

        clock_gettime(CLOCK_MONOTONIC, &wait_start);
        pthread_mutex_lock(&args->monitor->mutex); // clock_gettime() is called both before and right after acquiring the mutex -> pthread_mutex_lock() is a blocking call -> so the difference between wait_end and wait_start is exactly the time this thread spent blocked waiting
        clock_gettime(CLOCK_MONOTONIC, &wait_end);  // Note: we use CLOCK_MONOTONIC (not CLOCK_THREAD_CPUTIME_ID as in monitor.c)

        double battery = args->monitor->battery_level;
        pthread_mutex_unlock(&args->monitor->mutex);

        double wait_ms = (wait_end.tv_sec - wait_start.tv_sec) * 1000.0 + (wait_end.tv_nsec - wait_start.tv_nsec) / 1e6;
        if (wait_ms > WATCHDOG_MUTEX_WARN_MS) {
            fprintf(stderr, "[watchdog] waited %.2f ms on the telemetry mutex (possible priority inversion)\n", wait_ms);
        }
        if (battery < BATTERY_CRITICAL_PCT) {
            safety_trigger(args->safety, "critical battery");
        }

        timespec_add_ns(&next_tick, period_ns);
        sleep_until(&next_tick);
    }
    return NULL;
}
