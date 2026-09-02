cmake_minimum_required(VERSION 3.16)
# Prefer g++ when /usr/bin/c++ is clang pointing at a GCC install that
# lacks the libstdc++.so linker script (common on mixed clang/gcc images).
if(NOT CMAKE_CXX_COMPILER)
    if(EXISTS /usr/bin/g++-13)
        set(CMAKE_CXX_COMPILER /usr/bin/g++-13 CACHE FILEPATH "" FORCE)
    elseif(EXISTS /usr/bin/g++)
        set(CMAKE_CXX_COMPILER /usr/bin/g++ CACHE FILEPATH "" FORCE)
    endif()
endif()
project(LidarRayTraceCore LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(lidar_core
    core/src/ScanPattern.cpp
    core/src/CpuSceneBackend.cpp
    core/src/Simulator.cpp
    core/src/PointCloudIo.cpp
)
target_include_directories(lidar_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core/include)

add_executable(lidar_core_tests tests/test_core.cpp)
target_link_libraries(lidar_core_tests PRIVATE lidar_core)

add_executable(lidar_sim tools/lidar_sim_cli.cpp)
target_link_libraries(lidar_sim PRIVATE lidar_core)

enable_testing()
add_test(NAME lidar_core_tests COMMAND lidar_core_tests
         ${CMAKE_CURRENT_SOURCE_DIR}/scan_mode/mid360.csv)
