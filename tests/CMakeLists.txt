add_definitions(-DSYS_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/fake-sys/arena")
add_definitions(-DFAKE_SYS="${CMAKE_CURRENT_SOURCE_DIR}/fake-sys")
add_executable(api_tests api_tests.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ev3dev.cpp)
add_test(api_tests api_tests)
