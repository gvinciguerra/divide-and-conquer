cmake_minimum_required(VERSION 3.10)
project(SPM_Divide_and_Conquer)

set(CMAKE_CXX_STANDARD 14)

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
