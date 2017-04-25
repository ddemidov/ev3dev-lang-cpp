add_executable(api_tests
    api_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../ev3dev.cpp
    )

target_include_directories(api_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    )

target_compile_options(api_tests PRIVATE -std=c++0x)

target_compile_definitions(api_tests PRIVATE
    SYS_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/fake-sys/arena"
    FAKE_SYS="${CMAKE_CURRENT_SOURCE_DIR}/fake-sys"
    )

add_test(api_tests api_tests)
