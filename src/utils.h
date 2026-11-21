#ifndef UTILS_H
#define UTILS_H

#include <time.h>

// Adds ns nanoseconds to *t, handling the carry into seconds
void timespec_add_ns(struct timespec *t, long ns);

void sleep_until(const struct timespec *t);

// Active (spin) wait for about ms milliseconds: unlike usleep/nanosleep, the thread does NOT yield the CPU, it stays actually running for the whole duration
void busy_wait_ms(double ms);

#endif /* UTILS_H */
