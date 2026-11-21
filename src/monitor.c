#include <stdio.h>
#include "monitor.h"
#include "common.h"
#include "utils.h"

void *monitor_thread(void *arg) {
    monitor_args_t *args = (monitor_args_t *)arg;
    const long period_ns = 1000000000L / MONITOR_HZ;

    FILE *log_file = fopen("logs/telemetry.csv", "w");
    if (log_file) {
        fprintf(log_file, "time_s,x_m,y_m,altitude_m,roll_rad,pitch_rad,battery_pct\n");
    } else {
        perror("[monitor] unable to open logs/telemetry.csv (running from the project root?)");
    }

    struct timespec next_tick, start_time;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);
    start_time = next_tick;     // reference used to compute relative timestamps

    while (g_running) {
        drone_state_t st;
        drone_state_read(args->state, &st);

        // take the mutex -> drain battery (not below zero) -> write a log row -> busy-wait -> increment counter -> release

        pthread_mutex_lock(&args->monitor->mutex);
        args->monitor->battery_level -= BATTERY_DRAIN_PER_TICK;
        if (args->monitor->battery_level < 0.0) {
            args->monitor->battery_level = 0.0;
        }
        double battery = args->monitor->battery_level;

        if (log_file) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (now.tv_sec - start_time.tv_sec) + (now.tv_nsec - start_time.tv_nsec) / 1e9;
            fprintf(log_file, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\n", elapsed, st.x, st.y, st.z, st.roll, st.pitch, battery);
        }
        // ACTIVE wait (not a sleep) to stretch the critical section: this is what makes priority inversion observable
        busy_wait_ms(MONITOR_CRITICAL_SECTION_DELAY_MS);
        args->monitor->log_entries_written++;   // not strictly needed, just for completeness
        pthread_mutex_unlock(&args->monitor->mutex);

        timespec_add_ns(&next_tick, period_ns);
        sleep_until(&next_tick);
    }

    if (log_file) {
        fclose(log_file);
    }
    return NULL;
}
