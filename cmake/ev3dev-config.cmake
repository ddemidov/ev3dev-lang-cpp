# ev3dev-config.cmake - CMake configuration file for external
# projects.
#
# Use this by invoking
#
#   find_package(ev3dev)
#
# followed by
#
#   add_executable(myrobot myrobot.cpp)
#   target_link_libraries(myrobot ev3dev::ev3dev)
#
# The module defines ev3dev::ev3dev IMPORTED target that takes care of proper
# compile and link options.

include("${CMAKE_CURRENT_LIST_DIR}/ev3dev-targets.cmake")
message(STATUS "Found ev3dev")
