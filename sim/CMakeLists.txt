# Native host build of the OpenTrickler control code for simulation.
# This does NOT build firmware - it compiles the same control sources against
# host shims so the charge controller can be run and debugged on a PC.
cmake_minimum_required(VERSION 3.16)
project(ottrickler_sim C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  # -O0 -g by default: this exists to be stepped through in a debugger.
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(FW_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src)

# Real firmware sources, compiled unmodified.
set(FW_SOURCES
    ${FW_SRC}/charge_mode.cpp
    ${FW_SRC}/ai_tuning.c
    ${FW_SRC}/rest_ai_tuning.c
    ${FW_SRC}/profile.c
    ${FW_SRC}/common.c
    ${FW_SRC}/FloatRingBuffer.cpp
)

# Real-time build with the web GUI - this is the main one.
add_executable(ottrickler_sim
    ${FW_SOURCES}
    src/sim_plant.c
    src/sim_stubs.c
    src/sim_rt.cpp
    src/sim_http.cpp
    src/sim_main_gui.cpp
)

# Batch harness: runs N charges as fast as possible and prints statistics.
add_executable(ottrickler_batch
    ${FW_SOURCES}
    src/sim_plant.c
    src/sim_stubs.c
    src/sim_rt.cpp
    src/sim_http.cpp
    src/sim_main.cpp
)

# AI characterization harness: runs a full characterization through charge
# mode with a stall watchdog, then throws charges with the fitted model.
add_executable(ottrickler_ai_char
    ${FW_SOURCES}
    src/sim_plant.c
    src/sim_stubs.c
    src/sim_rt.cpp
    src/sim_http.cpp
    src/sim_ai_char.cpp
)

# Learn-from-every-throw prototype: PID charges whose stop thresholds are
# adjusted after each charge by src/throw_learner.c.
add_executable(ottrickler_learn
    ${FW_SOURCES}
    src/sim_plant.c
    src/sim_stubs.c
    src/sim_rt.cpp
    src/sim_http.cpp
    src/throw_learner.c
    src/sim_learn.cpp
)

foreach(tgt ottrickler_sim ottrickler_batch ottrickler_ai_char ottrickler_learn)
target_include_directories(${tgt} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/shims
    ${FW_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
endforeach()

find_package(Threads REQUIRED)

# profile.c uses assert() without including <assert.h>.
foreach(tgt ottrickler_sim ottrickler_batch ottrickler_ai_char ottrickler_learn)
  if(MSVC)
    # msvc_compat.h neutralises the GCC extensions the firmware headers use.
    # It must be parsed before any firmware header.
    # C++20 is required because charge_mode.cpp uses designated initialisers.
    set_property(TARGET ${tgt} PROPERTY CXX_STANDARD 20)
    target_compile_options(${tgt} PRIVATE
        /FI${CMAKE_CURRENT_SOURCE_DIR}/shims/msvc_compat.h
        /FIassert.h
        /W1 /wd4244 /wd4267 /wd4305 /wd4996)
    target_compile_definitions(${tgt} PRIVATE _CRT_SECURE_NO_WARNINGS)
  else()
    target_compile_options(${tgt} PRIVATE -include assert.h -w)
  endif()
endforeach()

target_link_libraries(ottrickler_sim PRIVATE Threads::Threads)
target_link_libraries(ottrickler_batch PRIVATE Threads::Threads)
target_link_libraries(ottrickler_ai_char PRIVATE Threads::Threads)
target_link_libraries(ottrickler_learn PRIVATE Threads::Threads)
if(NOT MSVC)
    target_link_libraries(ottrickler_sim PRIVATE m)
    target_link_libraries(ottrickler_batch PRIVATE m)
    target_link_libraries(ottrickler_ai_char PRIVATE m)
    target_link_libraries(ottrickler_learn PRIVATE m)
else()
    target_link_libraries(ottrickler_sim PRIVATE ws2_32)
    target_link_libraries(ottrickler_batch PRIVATE ws2_32)
    target_link_libraries(ottrickler_ai_char PRIVATE ws2_32)
    target_link_libraries(ottrickler_learn PRIVATE ws2_32)
endif()
