#
# Slim build: a single static library (asterx_core) + a single executable
# (asterx_driver).  No public C++ API, no install, no pimpl ABI.
#

set(ASTERX_CORE_SOURCES
    app_config.cpp
    sbf/crc16_xmodem.cpp
    sbf/frame_sync.cpp
    tcp_client.cpp
    receiver_config.cpp
    sbf_recorder.cpp)

add_library(asterx_core STATIC ${ASTERX_CORE_SOURCES})
add_library(asterx::core ALIAS asterx_core)

target_include_directories(asterx_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(asterx_core
    PUBLIC
        Threads::Threads
    PRIVATE
        spdlog::spdlog
        yaml-cpp)

target_compile_features(asterx_core PUBLIC cxx_std_17)

asterx_set_warnings(asterx_core)
asterx_set_sanitizers(asterx_core)

# ---------------------------------------------------------------------------
# Executable.
# ---------------------------------------------------------------------------
add_executable(asterx_driver main.cpp)

target_link_libraries(asterx_driver
    PRIVATE
        asterx::core
        spdlog::spdlog
        yaml-cpp)

asterx_set_warnings(asterx_driver)
asterx_set_sanitizers(asterx_driver)
