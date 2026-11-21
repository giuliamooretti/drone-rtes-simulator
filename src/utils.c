#include "utils.h"

void timespec_add_ns(struct timespec *t, long ns) {
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) {
        t->tv_nsec -= 1000000000L;
        t->tv_sec += 1;
    }
}

void sleep_until(const struct timespec *t) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t, NULL);
}

void busy_wait_ms(double ms) {
    // Note: CLOCK_THREAD_CPUTIME_ID, not CLOCK_MONOTONIC
    // CLOCK_THREAD_CPUTIME_ID measures the CPU time actually consumed by this thread, which only advances while the thread is really running
    // With CLOCK_MONOTONIC, if the thread gets preempted time still passes, and the function would exit "early" relative to the work actually done
    struct timespec start, now;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
    double elapsed_ms;
    do {
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now);
        elapsed_ms = (now.tv_sec - start.tv_sec) * 1000.0 + (now.tv_nsec - start.tv_nsec) / 1e6;
    } while (elapsed_ms < ms);
}
