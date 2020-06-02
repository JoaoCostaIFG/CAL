cmake_minimum_required(VERSION 3.15)
project(t4g3)

set(CMAKE_CXX_STANDARD 14)

if (WIN32)
# The line below is necessary on Windows OS
link_libraries(ws2_32 wsock32)
endif (WIN32)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_FLAGS "-Wall")
# add_definitions(-DDEBUG_ENABLED)

include_directories(lib)
include_directories(resources)
add_executable(t4g3 main.cpp GraphImporter.cpp GraphRenderer.cpp MeetingPoint.cpp lib/connection.cpp lib/graphviewer.cpp ProblemSolver.cpp)
