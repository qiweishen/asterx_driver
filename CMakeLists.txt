cmake_minimum_required(VERSION 3.16)

project(asterx_driver
    VERSION 0.1.0
    DESCRIPTION "AsteRx-i3 D Pro+ raw IMU + dual-antenna GNSS recorder (Ethernet, slim)"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

option(ASTERX_BUILD_TESTS  "Build unit tests"                   ON )
option(ASTERX_ENABLE_ASAN  "Enable AddressSanitizer"            OFF)
option(ASTERX_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer"  OFF)
option(ASTERX_ENABLE_TSAN  "Enable ThreadSanitizer"             OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)
include(Sanitizers)

# ---------------------------------------------------------------------------
# Dependencies (slim).
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)

# yaml-cpp: system package (libyaml-cpp-dev on Ubuntu 20.04 = 0.6.2).
find_package(yaml-cpp REQUIRED)

# spdlog: system package (libspdlog-dev on Ubuntu 20.04 = 1.5.0).
find_package(spdlog REQUIRED)

# Some environments expose system spdlog but a newer Homebrew fmt earlier in
# the include path via another dependency. The compiled spdlog target exports
# SPDLOG_FMT_EXTERNAL in that case, which can mix incompatible spdlog/fmt
# versions. Use spdlog header-only mode with its bundled fmt headers instead.
add_library(asterx_spdlog INTERFACE)
add_library(asterx::spdlog ALIAS asterx_spdlog)
target_compile_definitions(asterx_spdlog INTERFACE
    SPDLOG_HEADER_ONLY
    FMT_HEADER_ONLY)
foreach(_spdlog_target spdlog::spdlog_header_only spdlog::spdlog)
    if(TARGET ${_spdlog_target})
        get_target_property(_spdlog_includes ${_spdlog_target} INTERFACE_INCLUDE_DIRECTORIES)
        if(_spdlog_includes)
            target_include_directories(asterx_spdlog INTERFACE ${_spdlog_includes})
        endif()
    endif()
endforeach()

# Google Test for unit tests only (FetchContent — keeps build hermetic).
if(ASTERX_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
        GIT_SHALLOW    TRUE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# ---------------------------------------------------------------------------
# Subdirectories.
# ---------------------------------------------------------------------------
add_subdirectory(src)

if(ASTERX_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Summary.
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "----------------- asterx_driver ${PROJECT_VERSION} -----------------")
message(STATUS "  CMAKE_BUILD_TYPE      : ${CMAKE_BUILD_TYPE}")
message(STATUS "  CMAKE_CXX_COMPILER    : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  ASTERX_BUILD_TESTS    : ${ASTERX_BUILD_TESTS}")
message(STATUS "  ASTERX_ENABLE_ASAN    : ${ASTERX_ENABLE_ASAN}")
message(STATUS "  ASTERX_ENABLE_UBSAN   : ${ASTERX_ENABLE_UBSAN}")
message(STATUS "  ASTERX_ENABLE_TSAN    : ${ASTERX_ENABLE_TSAN}")
message(STATUS "------------------------------------------------------------------")
message(STATUS "")
