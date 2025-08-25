cmake_minimum_required(VERSION 3.10)
project(CircleCollisionSimulation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(CircleCollisionSimulation main.cpp)

target_include_directories(CircleCollisionSimulation PUBLIC raylib/include)
target_link_directories(CircleCollisionSimulation PUBLIC raylib)
target_link_libraries(CircleCollisionSimulation libraylib.so)
