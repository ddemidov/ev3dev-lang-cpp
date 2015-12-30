cmake_minimum_required(VERSION 2.8)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type")
endif()

project(ev3dev-lang-cpp)

set(EV3DEV_PLATFORM "EV3" CACHE STRING "Target ev3dev platform (EV3/RPI)")
set_property(CACHE EV3DEV_PLATFORM PROPERTY STRINGS "EV3" "RPI")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_library(ev3dev STATIC ev3dev.cpp)

function(add_ev3_executable target sources)
    add_executable(${target} ${sources})
    target_link_libraries(${target} ev3dev pthread)
endfunction()

enable_testing()

add_subdirectory(demos)
add_subdirectory(tests)
