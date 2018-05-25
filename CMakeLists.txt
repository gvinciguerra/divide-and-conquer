cmake_minimum_required(VERSION 3.10)
project(SPM_Divide_and_Conquer)

set(CMAKE_CXX_STANDARD 14)

set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Suppressing benchmark's tests" FORCE)
add_subdirectory(lib/benchmark)

add_executable("${PROJECT_NAME}" main.cpp include/divideandconquer.h include/workstealingpool.h)
target_link_libraries("${PROJECT_NAME}" benchmark)