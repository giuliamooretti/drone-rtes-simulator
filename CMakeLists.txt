cmake_minimum_required(VERSION 3.16)
project(drone_rtes C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)


find_package(Threads REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(drone_rtes
    src/main.c
    src/shared_state.c
    src/utils.c
    src/physics_thread.c
    src/state_estimator.c
    src/mission.c
    src/controller.c
    src/monitor.c
    src/watchdog.c
    src/cpu_load_thread.c
)

target_include_directories(drone_rtes PRIVATE ${MUJOCO_DIR}/include)
target_link_directories(drone_rtes PRIVATE ${MUJOCO_DIR}/lib)

target_link_libraries(drone_rtes
    PRIVATE
    Threads::Threads
    m
    mujoco
    glfw
)

