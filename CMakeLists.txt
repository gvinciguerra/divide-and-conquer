cmake_minimum_required(VERSION 3.10)
project(SPM_Divide_and_Conquer)

set(CMAKE_CXX_STANDARD 14)
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")
message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

# Uncomment the following two lines to output the optimisation report
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopt-info")
#set(CMAKE_VERBOSE_MAKEFILE ON)

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

# Benchmarks
set(EXE_NAME divide_and_conquer_benchmark)
add_executable("${EXE_NAME}" main.cpp include/divideandconquer.h include/workstealingpool.h include/simplepool.h)
target_link_libraries("${EXE_NAME}" benchmark)
