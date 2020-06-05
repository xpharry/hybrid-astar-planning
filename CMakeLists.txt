cmake_minimum_required(VERSION 2.8.9)

project(hybrid-astar)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(include maps)

file(GLOB SOURCES "src/*.cpp")

add_executable(hybrid_astar ${SOURCES})

target_link_libraries(hybrid_astar ${OpenCV_LIBS})
