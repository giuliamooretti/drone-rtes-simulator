#ifndef COMMON_H
#define COMMON_H

// Frequencies of the periodic tasks
#define PHYSICS_HZ 1000
#define ESTIMATOR_HZ 500
#define CONTROLLER_HZ 250
#define MISSION_HZ 10
#define MONITOR_HZ 2
#define WATCHDOG_HZ 100
#define CPU_LOAD_HZ 50   // 20ms period

// SCHED_FIFO priorities
#define PRIO_WATCHDOG 90
#define PRIO_PHYSICS 80
#define PRIO_CONTROLLER 70
#define PRIO_ESTIMATOR 50
#define PRIO_CPU_LOAD 40
#define PRIO_MISSION 30
#define PRIO_MONITOR 10

// How much the thread works
#define CPU_LOAD_BUSY_MS 6  // 30% duty cycle (also tried 50, 70)

// Safety thresholds
#define MAX_TILT_RAD 0.9        // about 51 degrees -> beyond this the watchdog forces the failsafe
#define CONTROL_DEADLINE_MS 8   // maximum deadline for one controller cycle
#define MAX_CONSECUTIVE_DEADLINE_MISSES 20
#define BATTERY_CRITICAL_PCT 5.0

#define NUM_ROTORS 4
#define WIND_FORCE_N 4.0

// Monitor thread parameters
#define BATTERY_DRAIN_PER_TICK 0.05        // drains in about 1000s
#define MONITOR_CRITICAL_SECTION_DELAY_MS 30

#define WATCHDOG_MUTEX_WARN_MS 10.0

// If 1, the monitor_data_t mutex uses PTHREAD_PRIO_INHERIT
// If 0, plain mutex
#define USE_PRIORITY_INHERITANCE 1

#endif /* COMMON_H */
