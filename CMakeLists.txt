project(MonteCarloLocalization)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 )

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

include_directories(
    /usr/include/python2.7 
    include
)

set(sources src/main.cpp 
            src/robot.cpp
            src/helper_functions.cpp
             
)

add_executable(MonteCarloLocalization ${sources})
target_link_libraries(MonteCarloLocalization python2.7)