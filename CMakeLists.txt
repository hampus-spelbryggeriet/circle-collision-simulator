cmake_minimum_required(VERSION 3.10)
project(CircleCollisionSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_definitions("-mavx2")

include_directories(inc)
add_executable(CircleCollisionSimulation
  src/collision.cpp
  src/main.cpp
  src/params.cpp
  src/simulation.cpp
)

target_include_directories(CircleCollisionSimulation PUBLIC raylib/include)
target_link_directories(CircleCollisionSimulation PUBLIC raylib)
target_link_libraries(CircleCollisionSimulation libraylib.so)
