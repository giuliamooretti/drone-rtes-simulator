# Drone RTES

A quadcopter flight simulator ([MuJoCo](https://mujoco.org/) physics) built as a **Real-Time
Embedded Systems** exercise: 7 POSIX threads with `SCHED_FIFO` priorities, deadlines, a
failsafe, and a live demo of **priority inversion** (and its fix via priority-inheritance
mutexes).

The drone (a [Skydio X2](https://www.skydio.com/skydio-x2) model) autonomously flies a square
path while, from the keyboard, you can inject rotor failures, wind and GPS loss to watch in
real time how the system handles them.

## Table of contents

- [Architecture](#architecture)
- [Setup](#setup)
- [Build and run](#build-and-run)
- [Keyboard controls](#keyboard-controls)
- [Priority inversion demo](#priority-inversion-demo)
- [Project structure](#project-structure)
- [License](#license)

## Architecture

Each feature lives in its own periodic thread, with its own frequency and `SCHED_FIFO` priority
(defined in [`src/common.h`](src/common.h)):

| Thread | File | Frequency | Priority | Role |
|---|---|---|---|---|
| Watchdog | [`watchdog.c`](src/watchdog.c) | 100 Hz | 90 (highest) | Checks deadline misses and attitude, triggers the failsafe |
| Physics | [`physics_thread.c`](src/physics_thread.c) | 1000 Hz | 80 | Steps the MuJoCo physics simulation (`mj_step`) |
| Controller | [`controller.c`](src/controller.c) | 250 Hz | 70 | PID on altitude/attitude/position, mixer over the 4 rotors |
| State estimator | [`state_estimator.c`](src/state_estimator.c) | 500 Hz | 50 | Reads MuJoCo sensors and updates the estimated state |
| CPU load | [`cpu_load_thread.c`](src/cpu_load_thread.c) | 50 Hz | 40 | Generates artificial CPU load for the demo |
| Mission | [`mission.c`](src/mission.c) | 10 Hz | 30 | Decides the next waypoint and publishes the setpoint |
| Monitor | [`monitor.c`](src/monitor.c) | 2 Hz | 10 (lowest) | Drains the simulated battery, writes telemetry to CSV |

The main thread (`main`, [`src/main.c`](src/main.c)) is not periodic: it creates the other
threads, handles keyboard input, and draws the MuJoCo/GLFW window.

State shared between threads ([`src/shared_state.c`](src/shared_state.c)/`.h`) uses a
`pthread_rwlock_t` for the estimated state (1 writer, N readers) and a `pthread_mutex_t` +
`pthread_cond_t` for the setpoint (producer/consumer). The keyboard-injected faults and the
safety flag are each protected by their own mutex.

**Safety chain:** `watchdog_thread` checks consecutive controller deadline misses, excessive
tilt, and critical battery; if any of these trip, it raises the `safety_flag_t`, which
`physics_thread` reads to zero out every rotor's thrust (failsafe). Thresholds live in
[`src/common.h`](src/common.h) (`MAX_TILT_RAD`, `CONTROL_DEADLINE_MS`,
`MAX_CONSECUTIVE_DEADLINE_MISSES`, `BATTERY_CRITICAL_PCT`).

## Setup

Dependencies: MuJoCo 3.x and GLFW (window/input library).

```bash
mkdir -p ~/opt && cd ~/opt
curl -sL -o mujoco.tar.gz https://github.com/google-deepmind/mujoco/releases/download/3.10.0/mujoco-3.10.0-linux-x86_64.tar.gz
tar xf mujoco.tar.gz

git clone --depth 1 https://github.com/glfw/glfw.git glfw-src
cmake -S glfw-src -B glfw-build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$HOME/opt/glfw
cmake --build glfw-build -j4 && cmake --install glfw-build
```

## Build and run

```bash
cmake -B build -DMUJOCO_DIR=$HOME/opt/mujoco-3.10.0 -DCMAKE_PREFIX_PATH=$HOME/opt/glfw
cmake --build build -j4

export LD_LIBRARY_PATH=$HOME/opt/mujoco-3.10.0/lib:$HOME/opt/glfw/lib:$LD_LIBRARY_PATH
./build/drone_rtes models/skydio_x2/scene.xml
```

The scene XML path argument is optional: without it, `models/skydio_x2/scene.xml` is used by
default. Run the program from the project root, since the monitor thread writes telemetry to
`logs/telemetry.csv` (a relative path).

For real-time scheduling (`SCHED_FIFO`) without running everything as root:

```bash
sudo setcap cap_sys_nice+ep ./build/drone_rtes
```

Without this capability, `create_rt_thread()` (in [`src/main.c`](src/main.c)) automatically
falls back to the default `SCHED_OTHER` scheduling: the program still works, but priorities
are not honored.

## Keyboard controls

| Key | Effect |
|---|---|
| `1`-`4` | Fail/restore the corresponding rotor |
| `W` | Toggle a wind gust |
| `G` | Simulate GPS loss/recovery |
| `R` | Reset all faults and the failsafe |
| `T` | Restart the simulation (physics reset) |

Rotor, wind, GPS, failsafe, altitude, position and battery status are shown as an overlay in
the simulation window.

## Priority inversion demo

The `monitor_data_t` mutex (battery/telemetry) is held for a while by the `monitor` thread via
a busy-wait (`MONITOR_CRITICAL_SECTION_DELAY_MS`), to make the phenomenon observable. The
`cpu_load_thread`, at medium priority, can prevent `monitor` (lower priority) from releasing
the mutex, which in turn makes `watchdog` (higher priority, but blocked on the same mutex) wait
longer than expected: classic priority inversion.

The `USE_PRIORITY_INHERITANCE` flag in [`src/common.h`](src/common.h) (0/1) enables or disables
the `PTHREAD_PRIO_INHERIT` protocol on that mutex:

- `0`: plain mutex -> with real `SCHED_FIFO` you observe the inversion (`watchdog` prints
  warnings whenever it waits on the mutex longer than `WATCHDOG_MUTEX_WARN_MS`).
- `1`: the mutex uses priority inheritance -> the thread holding it is temporarily boosted to
  the priority of the highest-priority thread waiting on it, eliminating the inversion.

Seeing the effect requires real `SCHED_FIFO` (see `setcap` above): with the `SCHED_OTHER`
fallback scheduling the demo isn't meaningful, since priorities aren't enforced.

## Project structure

```
.
├── CMakeLists.txt
├── src/                  # C sources (threads, shared state, utilities)
├── models/skydio_x2/     # MuJoCo drone model (MuJoCo Menagerie, own license)
├── logs/                 # telemetry CSV generated at runtime (not versioned)
└── build/                # CMake build output (not versioned)
```

## License

The code in this repository is distributed under the MIT license, see [`LICENSE`](LICENSE).

The 3D model in [`models/skydio_x2/`](models/skydio_x2) comes from the
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) and is distributed by
its original authors under the Apache 2.0 license (see `models/skydio_x2/LICENSE`).
