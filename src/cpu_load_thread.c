#include "cpu_load_thread.h"
#include "shared_state.h"   // only for g_running
#include "common.h"
#include "utils.h"

// this file exists solely to steal CPU in a controlled, predictable way
void *cpu_load_thread(void *arg) {  // the function signature still has to be void *(*)(void *) to be compatible with pthread_create -> it's an error to just drop the parameter
    (void)arg;  // tells the compiler: yes, I know I'm not using this
    const long period_ns = 1000000000L / CPU_LOAD_HZ;

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    while (g_running) {
        busy_wait_ms(CPU_LOAD_BUSY_MS);
        timespec_add_ns(&next_tick, period_ns);
        sleep_until(&next_tick);
    }
    return NULL;
}
