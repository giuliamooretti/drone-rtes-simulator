#define _GNU_SOURCE // for pthread_attr_setaffinity_np and CPU_SET/CPU_ZERO
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h> // handles the graphics window and keyboard input in this project

#include "common.h"
#include "shared_state.h"
#include "physics_thread.h"
#include "state_estimator.h"
#include "mission.h"
#include "controller.h"
#include "monitor.h"
#include "watchdog.h"
#include "cpu_load_thread.h"

#define DEFAULT_MODEL_PATH "models/skydio_x2/scene.xml"

// Display-only: avoids printing "-0.00" when the real value oscillates around zero due to numerical noise
static double display_clean(double v) {
    return fabs(v) < 0.005 ? 0.0 : v;
}

// Creates a thread with SCHED_FIFO priority
// pin_shared_core: forces the thread onto core 0 together with the other threads, so they actually contend for the same CPU and priority matters
// - physics is never pinned
// - Pinning only applies if SCHED_FIFO succeeds. Without it, the thread falls back to SCHED_OTHER (CFS, Completely Fair Scheduler), which ignores priorities.
static int create_rt_thread(pthread_t *thread, int priority, void *(*start_routine)(void *), void *arg, bool pin_shared_core) {
    cpu_set_t cpuset;       // defined in <sched.h> -> essentially a bitmask: one bit per CPU core, indicating whether that core is included in the set or not
    CPU_ZERO(&cpuset);      // clears all bits
    CPU_SET(0, &cpuset);    // sets only the bit corresponding to core 0

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (pin_shared_core) {
        pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    }

    struct sched_param sp;
    sp.sched_priority = priority;
    pthread_attr_setschedparam(&attr, &sp);

    int rc = pthread_create(thread, &attr, start_routine, arg);
    pthread_attr_destroy(&attr);

    if (rc != 0) {
        fprintf(stderr, "SCHED_FIFO unavailable, falling back to default scheduling for this thread\n");
        rc = pthread_create(thread, NULL, start_routine, arg);
    }
    return rc;
}

int main(int argc, char **argv) {
    const char *model_path;
    if (argc > 1) 
        model_path = argv[1];
    else
        model_path = DEFAULT_MODEL_PATH;

    char error[1000] = "";
    mjModel *model = mj_loadXML(model_path, NULL, error, sizeof(error));
    if (!model) {
        fprintf(stderr, "Error loading model '%s': %s\n", model_path, error);
        return 1;
    }
    mjData *data = mj_makeData(model);
    mj_forward(model, data); // computes initial sensordata/derivatives before starting the threads

    // Shared state structs
    drone_state_t state;
    setpoint_t setpoint;
    safety_flag_t safety;
    monitor_data_t monitor;
    control_health_t health;
    fault_flags_t faults;
    shared_state_init(&state, &setpoint, &safety, &monitor, &health, &faults);  // single function that initializes all the mutexes/rwlocks/condition variables inside each struct

    physics_args_t physics_args = { model, data, &safety, &faults };
    estimator_args_t estimator_args = { model, data, &state, &faults };
    mission_args_t mission_args = { &setpoint, &state };
    controller_args_t controller_args = { model, data, &state, &setpoint, &safety, &health };
    monitor_args_t monitor_args = { &monitor, &state };
    watchdog_args_t watchdog_args = { &health, &state, &safety, &monitor };

    pthread_t th_physics, th_estimator, th_mission, th_controller, th_monitor, th_watchdog, th_cpu_load;
    create_rt_thread(&th_physics, PRIO_PHYSICS, physics_thread, &physics_args, false);
    create_rt_thread(&th_estimator, PRIO_ESTIMATOR, estimator_thread, &estimator_args, true);
    create_rt_thread(&th_mission, PRIO_MISSION, mission_thread, &mission_args, true);
    create_rt_thread(&th_controller, PRIO_CONTROLLER, controller_thread, &controller_args, true);
    create_rt_thread(&th_monitor, PRIO_MONITOR, monitor_thread, &monitor_args, true);
    create_rt_thread(&th_watchdog, PRIO_WATCHDOG, watchdog_thread, &watchdog_args, true);
    create_rt_thread(&th_cpu_load, PRIO_CPU_LOAD, cpu_load_thread, NULL, true);

    // the MuJoCo/GLFW viewer must run on the main thread
    if (!glfwInit()) {
        fprintf(stderr, "Unable to initialize GLFW\n");
        return 1;
    }
    GLFWwindow *window = glfwCreateWindow(1200, 900, "Drone RTES", NULL, NULL);
    if (!window) {
        fprintf(stderr,
                "Unable to create the GLFW window (running as root with 'sudo' can't see your graphical server).\n"
                "No sudo needed for SCHED_FIFO: run 'sudo setcap cap_sys_nice+ep ./build/drone_rtes' once, then launch without sudo.\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window); // from now on, every OpenGL call made by this thread refers to the window
    glfwSwapInterval(1);    // needed for rendering

    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

    mjv_defaultFreeCamera(model, &cam);
    cam.type = mjCAMERA_TRACKING; // so it tracks the drone
    cam.trackbodyid = mj_name2id(model, mjOBJ_BODY, "x2");
    cam.distance = 3.0;
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    // Detects only the moment a key is pressed, not while it's held down
    bool key_was_down[GLFW_KEY_LAST + 1] = {0};

    // main is the only thread that writes faults (physics_thread and state_estimator only read them), so it can keep its own local copy
    bool local_rotor_failed[NUM_ROTORS] = {false};
    bool local_wind_active = false;
    bool local_gps_lost = false;

    while (!glfwWindowShouldClose(window)) {    
        static const int rotor_keys[NUM_ROTORS] = {GLFW_KEY_1, GLFW_KEY_2, GLFW_KEY_3, GLFW_KEY_4}; //  maps the rotor index to the corresponding GLFW key
        bool down;
        for (int i = 0; i < NUM_ROTORS; i++) {
            down = glfwGetKey(window, rotor_keys[i]) == GLFW_PRESS;
            if (down && !key_was_down[rotor_keys[i]]) {     // pressed now but wasn't on the previous frame
                local_rotor_failed[i] = !local_rotor_failed[i];     // toggles the state
                fault_set_rotor(&faults, i, local_rotor_failed[i]);     // propagates the new state to the shared faults struct -> read by physics
                printf("[fault] rotor %d %s\n", i + 1, local_rotor_failed[i] ? "FAILED" : "restored");
            }
            key_was_down[rotor_keys[i]] = down;
        }
        // same for wind
        down = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
        if (down && !key_was_down[GLFW_KEY_W]) {
            local_wind_active = !local_wind_active;
            fault_set_wind(&faults, local_wind_active);
            printf("[fault] wind gust %s\n", local_wind_active ? "ACTIVE" : "disabled");
        }
        key_was_down[GLFW_KEY_W] = down;

        // gps
        down = glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS;
        if (down && !key_was_down[GLFW_KEY_G]) {
            local_gps_lost = !local_gps_lost;
            fault_set_gps_lost(&faults, local_gps_lost);
            printf("[fault] GPS %s\n", local_gps_lost ? "LOST" : "restored");
        }
        key_was_down[GLFW_KEY_G] = down;

        // R
        down = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
        if (down && !key_was_down[GLFW_KEY_R]) {
            fault_clear_all(&faults);
            safety_clear(&safety);
            for (int i = 0; i < NUM_ROTORS; i++) {
                local_rotor_failed[i] = false;
            }
            local_wind_active = false;
            local_gps_lost = false;
            printf("[fault] full reset (faults cleared, failsafe disabled)\n");
        }
        key_was_down[GLFW_KEY_R] = down;

        // T
        down = glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS;
        if (down && !key_was_down[GLFW_KEY_T]) {

            pthread_mutex_lock(&sim_mutex);
            mj_resetData(model, data);
            mj_forward(model, data);
            pthread_mutex_unlock(&sim_mutex);

            fault_clear_all(&faults);
            safety_clear(&safety);
            for (int i = 0; i < NUM_ROTORS; i++) {
                local_rotor_failed[i] = false;
            }
            local_wind_active = false;
            local_gps_lost = false;
            printf("[sim] simulation restarted\n");
        }
        key_was_down[GLFW_KEY_T] = down;

        // rendering
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        drone_state_t st;
        drone_state_read(&state, &st);

        // re-centers the area covered by the shadow map on the drone's current position
        model->stat.center[0] = st.x;
        model->stat.center[1] = st.y;


        pthread_mutex_lock(&sim_mutex);
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        pthread_mutex_unlock(&sim_mutex);

        mjr_render(viewport, &scn, &con);

        mjr_overlay(mjFONT_SHADOW, mjGRID_TOPLEFT, viewport, "Controls\n1-4 rotor failure\nW wind\nG GPS\nR reset faults\nT restart", "", &con);

        double battery = monitor_get_battery(&monitor);

        char status_text[320];
        snprintf(status_text, sizeof(status_text),
                 "Rotors: %s %s %s %s\nWind: %s\nGPS: %s\n%s\n"
                 "Altitude: %.2f m\nPosition: (%.2f, %.2f)\nBattery: %.1f%%",
                 local_rotor_failed[0] ? "1=X" : "1=ok",
                 local_rotor_failed[1] ? "2=X" : "2=ok",
                 local_rotor_failed[2] ? "3=X" : "3=ok",
                 local_rotor_failed[3] ? "4=X" : "4=ok",
                 local_wind_active ? "active" : "none",
                 local_gps_lost ? "lost" : "present",
                 safety_is_active(&safety) ? "FAILSAFE ACTIVE" : "normal flight",
                 st.z, display_clean(st.x), display_clean(st.y), battery);
        mjr_overlay(mjFONT_SHADOW, mjGRID_TOPRIGHT, viewport, status_text, "", &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Orderly shutdown
    g_running = 0;
    pthread_join(th_physics, NULL);
    pthread_join(th_estimator, NULL);
    pthread_join(th_mission, NULL);
    pthread_join(th_controller, NULL);
    pthread_join(th_monitor, NULL);
    pthread_join(th_watchdog, NULL);
    pthread_join(th_cpu_load, NULL);

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwTerminate();

    shared_state_destroy(&state, &setpoint, &safety, &monitor, &health, &faults);
    mj_deleteData(data);
    mj_deleteModel(model);

    return 0;
}
