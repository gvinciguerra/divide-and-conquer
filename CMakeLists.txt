cmake_minimum_required(VERSION 3.10)
project(SPM_Divide_and_Conquer)

set(CMAKE_CXX_STANDARD 14)

# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

# Google Benchmark
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Skip Google Benchmark tests" FORCE)
set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "Skip Google Benchmark install")
add_subdirectory(lib/benchmark)

# Fastflow
set(BUILD_TESTS OFF CACHE BOOL "Skip FastFlow tests")
set(BUILD_EXAMPLES OFF CACHE BOOL "Skip FastFlow examples")
include_directories(lib/fastflow)

add_executable("${PROJECT_NAME}" main.cpp include/divideandconquer.h include/workstealingpool.h include/simplepool.h)
target_link_libraries("${PROJECT_NAME}" benchmark)
