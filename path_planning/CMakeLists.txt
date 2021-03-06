project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
    src/main.cpp
    src/path_planner.cpp
    src/path_planner.h
    src/vehicle.h
    src/utils.cpp
    src/utils.h
    src/cost_function.h
    src/cost_function.cpp
    src/json.hpp
    src/constants.h
    src/spline.h)

set(CMAKE_CXX_STANDARD 14)
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
