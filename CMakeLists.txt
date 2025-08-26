cmake_minimum_required(VERSION 3.10)
project(CircleCollisionSimulation)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_definitions("-mavx2 -mfma")

include_directories(inc)
add_executable(CircleCollisionSimulation
  src/main.cpp
  src/params.cpp
  src/simulation.cpp
)

target_include_directories(CircleCollisionSimulation PUBLIC raylib/include)
target_link_directories(CircleCollisionSimulation PUBLIC raylib)
target_link_libraries(CircleCollisionSimulation libraylib.so)
