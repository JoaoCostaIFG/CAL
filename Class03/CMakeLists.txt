cmake_minimum_required(VERSION 3.10)
project(CAL_FP03)

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(lib/googletest-master)
include_directories(lib/googletest-master/googletest/include)
include_directories(lib/googletest-master/googlemock/include)

set(CMAKE_CXX_FLAGS "-Wall -O3")
# set(WORKING_DIRECTORY FILES)

add_executable(CAL_FP03 main.cpp Tests/tests.cpp Tests/NearestPoints.cpp Tests/Point.cpp)

target_link_libraries(CAL_FP03 gtest gtest_main)
