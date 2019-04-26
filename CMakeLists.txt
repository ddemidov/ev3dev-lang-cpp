cmake_minimum_required(VERSION 3.1)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type")
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ev3dev-lang-cpp)

# Is this used directly or via add_subdirectory()
set(EV3DEV_LANG_CPP_MASTER_PROJECT OFF)
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(EV3DEV_LANG_CPP_MASTER_PROJECT ON)
endif()

set(EV3DEV_PLATFORM "EV3" CACHE STRING "Target ev3dev platform (EV3/BRICKPI/BRICKPI3/PISTORMS)")
set_property(CACHE EV3DEV_PLATFORM PROPERTY STRINGS "EV3" "BRICKPI" "BRICKPI3" "PISTORMS")

add_library(ev3dev STATIC ev3dev.cpp)
add_library(ev3dev::ev3dev ALIAS ev3dev) # to match exported target

target_include_directories(ev3dev PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
    )

target_compile_definitions(ev3dev PUBLIC
    _GLIBCXX_USE_NANOSLEEP
    EV3DEV_PLATFORM_${EV3DEV_PLATFORM}
    )
target_link_libraries(ev3dev PUBLIC pthread)

function(add_ev3_executable target sources)
    add_executable(${target} ${sources})
    target_link_libraries(${target} ev3dev)
endfunction()

if (EV3DEV_LANG_CPP_MASTER_PROJECT)
    enable_testing()
    add_subdirectory(tests)

    add_subdirectory(demos)

    #----------------------------------------------------------------------
    # Install the library, header, and cmake configuration
    #----------------------------------------------------------------------
    install(FILES ev3dev.h DESTINATION include)
    install(TARGETS ev3dev EXPORT ev3devTargets
        LIBRARY DESTINATION  lib
        ARCHIVE DESTINATION  lib
        INCLUDES DESTINATION include
        )

    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ev3dev-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake/ev3dev-config.cmake"
        COPYONLY
        )

    export(EXPORT ev3devTargets
        FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/ev3dev-targets.cmake"
        NAMESPACE ev3dev::
        )

    export(PACKAGE ev3dev)

    install(EXPORT ev3devTargets
        FILE ev3dev-targets.cmake
        NAMESPACE ev3dev::
        DESTINATION share/ev3dev/cmake
        )

    install(
        FILES cmake/ev3dev-config.cmake
        DESTINATION share/ev3dev/cmake
        )
endif()


