#
# Slim unit tests.  Two suites:
#   asterx_test_crc16_xmodem  - CRC sanity (canonical vectors)
#   asterx_test_frame_sync    - SBF byte-machine
#

include(GoogleTest)

function(asterx_add_test name)
    cmake_parse_arguments(_T "" "" "SRCS;LIBS" ${ARGN})

    add_executable(${name} ${_T_SRCS})
    target_link_libraries(${name}
        PRIVATE
            asterx::core
            GTest::gtest
            GTest::gtest_main
            ${_T_LIBS})

    # Tests are allowed to include private src/ headers.
    target_include_directories(${name}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/src)

    target_compile_features(${name} PRIVATE cxx_std_17)
    asterx_set_warnings(${name})
    asterx_set_sanitizers(${name})

    gtest_discover_tests(${name}
        DISCOVERY_TIMEOUT 30
        PROPERTIES
            LABELS unit)
endfunction()

asterx_add_test(asterx_test_crc16_xmodem
    SRCS test_crc16_xmodem.cpp)

asterx_add_test(asterx_test_receiver_config
    SRCS test_receiver_config.cpp)

asterx_add_test(asterx_test_app_config
    SRCS test_app_config.cpp)

add_subdirectory(unit/sbf)
