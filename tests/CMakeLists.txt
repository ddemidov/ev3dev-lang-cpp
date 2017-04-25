add_executable(api_tests api_tests.cpp)

target_compile_definitions(api_tests PRIVATE
    SYS_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/fake-sys/arena"
    FAKE_SYS="${CMAKE_CURRENT_SOURCE_DIR}/fake-sys"
    )
target_link_libraries(api_tests ev3dev)

add_test(api_tests api_tests)
