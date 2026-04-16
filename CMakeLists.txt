cmake_minimum_required(VERSION 3.10)
project(sim_app)

set(CMAKE_CXX_STANDARD 17)

add_executable(sim_app code.cpp)

target_link_libraries(sim_app gpiod pthread)